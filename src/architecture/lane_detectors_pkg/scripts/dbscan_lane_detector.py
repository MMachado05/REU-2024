#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from lane_detectors_pkg.cfg import DBScanLaneDetectorConfig  # packageName.cfg
from geometry_msgs.msg import Twist
import cv2 as cv
import numpy as np
from sklearn.cluster import DBSCAN
import threading


class DBScanLaneDetector:
    """
    Density-based scan lane detector node.

    Logic taken from Beñat's "advanced_dbw_follow_lane.py" code.
    """

    preprocessed_img_subscriber: rospy.Subscriber
    center_offset_publisher: rospy.Publisher
    dyn_rcfg_srv: Server

    display_desired_twist_image: bool

    hough_thresh: int
    min_line_length: int
    max_line_gap: int

    min_slope_abs: float
    extension_factor: float
    extension_candidate_max: int

    percent_downsample: int
    epsilon: int

    rosimg_cv_bridge: CvBridge
    offset_message: Twist
    num_line_last_seen: int

    generated_houghlines: bool
    found_white_pixels: bool


    # ------------------------------------------------
    # ------- Internal state-related functions -------
    # ------------------------------------------------
    def __init__(self):
        """
        Initializes the node.
        """
        # Node architecture
        rospy.init_node("dbscan_lane_detector", anonymous=True)

        self.preprocessed_img_subscriber = rospy.Subscriber(
            rospy.get_param("~img_in_topic"), Image, self._image_callback
        )
        self.center_offset_publisher = rospy.Publisher(
            rospy.get_param("~desired_twist_out_topic"), Twist, queue_size=1
        )
        self.dyn_rcfg_srv = Server(
            DBScanLaneDetectorConfig, self._dynamic_reconfig_callback
        )

        # Initially-set dynamic reconfigure parameters
        self.display_desired_twist_image = False

        self.hough_thresh = 20
        self.min_line_length = 10
        self.max_line_gap = 10

        self.min_slope_abs = 2.5
        self.extension_factor = 2.0
        self.extension_candidate_max = 100

        self.percent_downsample = 10
        self.epsilon = 70

        # Misc.
        self.rosimg_cv_bridge = CvBridge()
        self.offset_message = Twist()
        self.num_line_last_seen = 0

        self.generated_houghlines = True
        self.found_white_pixels = True

        self.desired_twist_x = 0
        self.desired_twist_y = 0

        self.image_lock = threading.Lock()
        self.ros_image = None

        self.detection_thread = threading.Thread(target=self._run_detection_loop)
        self.detection_thread.daemon = True
        self.detection_thread.start()

        self.publisher_thread = threading.Thread(target=self.publisher)
        self.publisher_thread.daemon = True
        self.publisher_thread.start()

        # Begin lane detection
        rospy.spin()

    def publisher(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            self.offset_message.angular.x = self.desired_twist_x
            self.offset_message.angular.y = self.desired_twist_y
            self.offset_message.angular.z = 1
            self.center_offset_publisher.publish(self.offset_message)
            rate.sleep()

    def _dynamic_reconfig_callback(self, config, _):
        """
        Callback function for dynamic reconfigure.

        Parameters
        ----------
        config : DBScanLaneDetectorConfig
            The new dynamic reconfigure parameters.
        """
        self.display_desired_twist_image = config.display_desired_twist_image

        self.hough_thresh = config.thresh
        self.min_line_length = config.min_line_length
        self.max_line_gap = config.max_line_gap

        self.min_slope_abs = config.min_slope_abs
        self.extension_factor = config.extension_factor
        self.extension_candidate_max = config.extension_candidate_max

        self.percent_downsample = config.percent_downsample
        self.epsilon = config.epsilon

        return config

    # ---------------------------------------
    # ----------- Lane detection ------------
    # ---------------------------------------
    def _image_callback(self, ros_image: Image) -> None:
        with self.image_lock:
            self.ros_image = ros_image

    def _run_detection_loop(self):
        rate = rospy.Rate(50)  
        while not rospy.is_shutdown():
            with self.image_lock:
                if self.ros_image is not None:
                    self._process_image(self.ros_image)
            rate.sleep()

    def _process_image(self, ros_image):
        """
        Detects a lane in the image and publishes the offset between the desired
        midpoint, and the current midpoint.

        The expected values to be published are angular x and y speeds within absolute
        values of 1. In the case of an exceptional situation, the z angular velocity
        will contain useful information:
        * 1: No lane lines were detected.
        * 2: Only one lane line was detected.
        * 3: There was some other exceptional error.

        Paramaters
        ----------
        ros_image : Image
            The preprocessed image to detect the lane in.

        
        """
        try:
            grayscale_cv_image = self.rosimg_cv_bridge.imgmsg_to_cv2(
                ros_image
            )  # passthrough
            grayscale_cv_image = grayscale_cv_image.copy()
            color_cv_image = cv.cvtColor(grayscale_cv_image, cv.COLOR_GRAY2BGR)
        except CvBridgeError as e:
            rospy.logerr(f"dbscan_lane_detector - CvBridge Error: {e}")
            self.offset_message.angular.z = 3
            self.center_offset_publisher.publish(self.offset_message)
            return

        houghlines = cv.HoughLinesP(
            grayscale_cv_image,
            3,
            np.pi / 180,
            threshold=self.hough_thresh,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap,
        )

        # Generate a B/W image with only validated hough lane lines
        houghlines_only_image = np.zeros_like(grayscale_cv_image)
        if houghlines is None:
            if self.display_desired_twist_image:
                cv.imshow("Twist message visualization", grayscale_cv_image)
                cv.waitKey(1)
            if self.generated_houghlines:
                rospy.logerr("dbscan_lane_detector - Unable to generate hough lines.")
                self.generated_houghlines = False
            self.offset_message.angular.z = 1
            self.center_offset_publisher.publish(self.offset_message)
            return

        for line in houghlines:
            x1, y1, x2, y2 = line[0]
            slope = float("inf") if x2 == x1 else (y2 - y1) / (x2 - x1)
            length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

            # If this works, I'm killing myself :)
            # Welp... tell the maids to clean my room for Michael :(
            if abs(slope) > self.min_slope_abs:
                if length < self.extension_candidate_max:
                    x1 = int(x1 - (x2 - x1) * self.extension_factor)
                    y1 = int(y1 - (y2 - y1) * self.extension_factor)
                    x2 = int(x2 + (x2 - x1) * self.extension_factor)
                    y2 = int(y2 + (y2 - y1) * self.extension_factor)
                cv.line(houghlines_only_image, (x1, y1), (x2, y2), (255, 255, 255), 2)

        # Collect white points for DBScan
        white_points = np.column_stack(np.where(houghlines_only_image > 0))
        if len(white_points) == 0:
            if self.display_desired_twist_image:
                cv.imshow("Twist message visualization", grayscale_cv_image)
                cv.waitKey(1)
            if self.found_white_pixels:
                rospy.logerr("dbscan_lane_detector - Unable to detect white pixels.")
                self.found_white_pixels = False
            self.offset_message.angular.z = 1
            self.center_offset_publisher.publish(self.offset_message)
            return

        downsample_step = int(100 / self.percent_downsample)
        white_points = white_points[::downsample_step]

        dbscan = DBSCAN(eps=self.epsilon, min_samples=3)
        cluster_labels = dbscan.fit_predict(white_points)

        # Prepare lane line clusters
        unique_labels = np.unique(cluster_labels)
        potential_lane_line_cluster_labels = []
        for label in unique_labels:
            cluster_points = white_points[cluster_labels == label]
            centroid_y = np.max(cluster_points[:, 0])
            if (
                label != -1 and len(cluster_points) >= 10
            ):  # Ignore points labeled as noise
                # TODO: Make this dynamically reconfigurable (maybe)
                potential_lane_line_cluster_labels.append((label, centroid_y))

        sorted_clusters = sorted(
            potential_lane_line_cluster_labels, key=lambda x: x[1], reverse=True
        )[
            :2
        ]  # Want the two biggest clusters (potential lane lines)
        candidate_labels = [sorted_clusters[i][0] for i in range(len(sorted_clusters))]

        lane_line_centroids_x = []
        lane_line_centroids_y = []
        lane_line_centroids = 0

        label_to_color = {}
        if self.display_desired_twist_image:
            lane_line_colors = [(0, 255, 0), (0, 0, 255)]
            label_to_color = {
                label: lane_line_colors[i] for i, label in enumerate(candidate_labels)
            }  # Set color labeling variables if we're displaying the final image

        for label in candidate_labels:
            lane_line_points = white_points[cluster_labels == label]
            lane_line_points = lane_line_points[np.argsort(lane_line_points[:, 1])]

            if self.display_desired_twist_image:
                for point in lane_line_points:
                    cv.circle(
                        color_cv_image,
                        (int(point[1]), int(point[0])),
                        5,
                        label_to_color[label],
                        -1,
                    )

            lane_line_centroids_x.append(np.mean(lane_line_points[:, 1]))
            lane_line_centroids_y.append(np.mean(lane_line_points[:, 0]))
            lane_line_centroids += 1

        # Calculate information from candidate lane lines
        if lane_line_centroids == 0:
            if self.num_line_last_seen != 0:
                rospy.logerr("dbscan_lane_detector - I can't see any lane lines!")
                self.num_line_last_seen = 0
            self.offset_message.angular.z = 1
            self.center_offset_publisher.publish(self.offset_message)
            if self.display_desired_twist_image:
                cv.imshow("Twist message visualization", color_cv_image)
                cv.waitKey(1)
            return

        centroid_avg_midpoint_x = int(np.mean(lane_line_centroids_x))
        centroid_avg_midpoint_y = int(np.mean(lane_line_centroids_y))

        rows, cols = grayscale_cv_image.shape
        image_midpoint_x = cols // 2
        height = rows

        # Twist method calculation depends on how the x centroid was derived
        if lane_line_centroids == 1:
            if self.num_line_last_seen != 1:
                rospy.logwarn("dbscan_lane_detector - I can only see one lane line!")
                self.num_line_last_seen = 1

            x_side = -1 if centroid_avg_midpoint_x < image_midpoint_x else 1

            self.desired_twist_x = -(
                x_side
                * (
                    (image_midpoint_x - abs(centroid_avg_midpoint_x - image_midpoint_x))
                    / image_midpoint_x
                )
            )
        else:
            if self.num_line_last_seen != 2:
                rospy.loginfo("dbscan_lane_detector - Two lane lines found.")
                self.num_line_last_seen = 2

            self.desired_twist_x = (
                centroid_avg_midpoint_x - image_midpoint_x
            ) / image_midpoint_x
        self.desired_twist_y = (height - centroid_avg_midpoint_y) / height

        if self.display_desired_twist_image:
            desired_midpoint_x = int(
                image_midpoint_x + (self.desired_twist_x * image_midpoint_x)
            )
            desired_midpoint_y = centroid_avg_midpoint_y
            cv.arrowedLine(
                color_cv_image,
                (image_midpoint_x, rows),
                (desired_midpoint_x, desired_midpoint_y),
                (255, 255, 255),
                2,
            )
            cv.imshow("Twist message visualization", color_cv_image)
            cv.waitKey(1)

if __name__ == "__main__":
    try:
        DBScanLaneDetector()
    except rospy.ROSInterruptException:
        pass
