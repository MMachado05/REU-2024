#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import cv2 as cv
import numpy as np
from sklearn.cluster import KMeans

from dynamic_reconfigure.server import Server
from lane_detectors_pkg.cfg import LargestContourLaneDetectorConfig  # packageName.cfg


class LargestContourLaneDetector:
    """
    K-means lane detector node.

    Uses Beñat's "kmeans_lane_follow.py" code.
    """

    preprocessed_img_subscriber: rospy.Subscriber
    center_offset_publisher: rospy.Publisher
    dyn_rcfg_srv: Server

    display_desired_twist_image: bool

    minimum_lane_gap: int

    # TODO: May need stuff

    rosimg_cv_bridge: CvBridge
    offset_message: Twist

    left_lane_x: int
    right_lane_x: int

    could_not_kmeans: bool

    # ------------------------------------------------
    # ------- Internal state-related functions -------
    # ------------------------------------------------
    def __init__(self):
        """
        Initializes the node.
        """
        # Node architecture
        rospy.init_node("kmeans_lane_detector", anonymous=True)

        self.preprocessed_img_subscriber = rospy.Subscriber(
            rospy.get_param("~img_in_topic"), Image, self._find_lane
        )
        self.center_offset_publisher = rospy.Publisher(
            rospy.get_param("~desired_twist_out_topic"), Twist, queue_size=1
        )
        self.dyn_rcfg_srv = Server(
            LargestContourLaneDetectorConfig, self._dynamic_reconfig_callback
        )

        # Initially-set dynamic reconfigure parameters
        self.display_desired_twist_image = False

        self.minimum_lane_gap = 100

        # Misc.
        self.rosimg_cv_bridge = CvBridge()
        self.offset_message = Twist()
        self.num_line_last_seen = 0

        self.left_lane_x = -1
        self.right_lane_x = -1

        self.could_not_kmeans = False

    def _dynamic_reconfig_callback(self, config, _):
        """
        Callback function for dynamic reconfigure.

        Parameters
        ----------
        config : DBScanLaneDetectorConfig
            The new dynamic reconfigure parameters.
        """
        self.display_desired_twist_image = config.display_desired_twist_image

        self.minimum_lane_gap = config.minimum_lane_gap

        return config

    # ---------------------------------------
    # ----------- Lane detection ------------
    # ---------------------------------------
    def _find_lane(self, ros_image: Image) -> None:
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
            rospy.logerr(f"kmeans_lane_detector:116 - CvBridge Error: {e}")
            self.offset_message.angular.z = 3
            self.center_offset_publisher.publish(self.offset_message)
            return

        rows, cols = grayscale_cv_image.shape
        image_midpoint_x = cols // 2
        image_midpoint_y = rows // 2

        if self.left_lane_x == -1:
            self.left_lane_x = image_midpoint_x - (image_midpoint_x // 2)
            rospy.loginfo(
                "kmeans_lane_detector:128 - Setting left lane x to default value."
            )
        if self.right_lane_x == -1:
            self.right_lane_x = image_midpoint_x + (image_midpoint_x // 2)
            rospy.loginfo(
                "kmeans_lane_detector:133 - Setting right lane x to default value."
            )

        cv.line(
            color_cv_image,
            (0, image_midpoint_y),
            (cols, image_midpoint_y),
            (0, 255, 0),
            2,
        )
        
        # TODO: Shit myself!!1!!11!!!11!

        white_points_on_line = np.where(grayscale_cv_image[image_midpoint_y, :] == 255)[
            0  # We only want the x coordinate, which as at the 0th index
        ]
        # Scale the image to only have points go from -1 to 1
        white_points_on_line = white_points_on_line.reshape(-1, 1)

        for point in white_points_on_line.flatten():
            cv.circle(color_cv_image, (point, image_midpoint_y), 5, (0, 0, 255), -1)

        # Need minimum 1 sample (whatever that means) for KMeans to work
        if len(white_points_on_line) >= 2:
            if self.could_not_kmeans:
                self.could_not_kmeans = False
                rospy.loginfo("kmeans_lane_detector:157 - KMeans successfully performed.")
            kmean_clusters = KMeans(n_init="auto", n_clusters=2).fit(
                white_points_on_line
            )
            cluster_centers = kmean_clusters.cluster_centers_.flatten()

            cluster_centers.sort()
            if abs(cluster_centers[0] - cluster_centers[1]) > self.minimum_lane_gap:
                self.left_lane_x = cluster_centers[0]
                self.right_lane_x = cluster_centers[1]
            else:
                single_point_avg = np.mean(cluster_centers)
                if single_point_avg > image_midpoint_x:
                    self.right_lane_x = single_point_avg
                else:
                    self.left_lane_x = single_point_avg
        elif not self.could_not_kmeans:
            rospy.logwarn(
                "kmeans_lane_detector:175 - Could not perform KMeans; resorting to old values."
            )
            self.could_not_kmeans = True

        desired_midpoint_x = int(np.mean([self.left_lane_x, self.right_lane_x]))
        desired_midpoint_y = image_midpoint_y

        cv.arrowedLine(
            color_cv_image,
            (image_midpoint_x, rows - 1),
            (desired_midpoint_x, desired_midpoint_y),
            (255, 255, 255),
            2,
        )

        desired_twist_x = (desired_midpoint_x - image_midpoint_x) / image_midpoint_x
        desired_twist_y = 0.5

        self.offset_message.angular.x = desired_twist_x
        self.offset_message.angular.y = desired_twist_y
        self.offset_message.angular.z = 0

        if self.display_desired_twist_image:
            cv.imshow("Twist message visualization", color_cv_image)
            cv.waitKey(1)

        self.center_offset_publisher.publish(self.offset_message)


if __name__ == "__main__":
    try:
        detector = LargestContourLaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
