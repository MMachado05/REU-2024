#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sklearn.cluster import DBSCAN
import cv2 as cv
import numpy as np
import math

# TODO: Add any imports that the implementation will need

from dynamic_reconfigure.server import Server
from lane_detectors_pkg.cfg import BirdseyeLaneDetectorConfig  # packageName.cfg


class BirdseyeLaneDetector:
    """
    Birds Eye lane detector node.

    Uses "bird_eye_dbscan.py" code.
    """

    preprocessed_img_subscriber: rospy.Subscriber
    center_offset_publisher: rospy.Publisher
    dyn_rcfg_srv: Server

    display_desired_twist_image: bool

    # TODO: Add any missing parameters that the implementation will need

    # TODO: May need stuff

    rosimg_cv_bridge: CvBridge
    offset_message: Twist

    # ------------------------------------------------
    # ------- Internal state-related functions -------
    # ------------------------------------------------
    def __init__(self):
        """
        Initializes the node.
        """
        # Node architecture
        rospy.init_node("birds_eye", anonymous=True)

        self.preprocessed_img_subscriber = rospy.Subscriber(
            rospy.get_param("~img_in_topic"), Image, self._find_lane
        )
        self.center_offset_publisher = rospy.Publisher(
            rospy.get_param("~desired_twist_out_topic"), Twist, queue_size=1
        )
        self.dyn_rcfg_srv = Server(
            BirdseyeLaneDetectorConfig, self._dynamic_reconfig_callback
        )

        # Initially-set dynamic reconfigure parameters
        self.display_desired_twist_image = False

        self.minimum_lane_gap = 100
        
        #dynamic reconfiguration params
        self.thresh = 200
        self.pers = 0.17
        self.width = 0.5
        self.crop_hor = 0.5
        self.crop_ver = 0.5
        self.crop_orig = 0.5

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
        

        # FIXME: Need to implement all of this
        
        self.thresh = config.thresh
        self.pers = config.pers
        self.width = config.pers_width
        self.crop_hor = config.crop_lines_hor
        self.crop_ver = config.crop_lines_ver
        self.crop_orig = config.crop_orig
        

        return config

    def change_perspective(self, img):
        rows, cols, _ = img.shape
        
        p1 = [0, 0]
        p2 = [int(cols * self.width), 0]
        p3 = [int(cols * self.width) - int(cols * self.pers), rows]
        p4 = [int(cols * self.pers), rows]
        
        per1 = np.float32([[0, 0], [cols, 0], [cols, rows], [0, rows]])
        per2 = np.float32([p1, p2, p3, p4])
        
        matrix = cv.getPerspectiveTransform(per1, per2)
        result = cv.warpPerspective(img, matrix, (int(cols * self.width), rows))
        return result
    
    def crop_image(self, image):
        # crop image
        if len(image.shape) < 3:
            rows, cols = image.shape
        else:
            rows, cols, ch = image.shape
        left_bound = cols//2 - int(cols * self.crop_hor / 2)
        right_bound = cols//2 + int(cols * self.crop_hor / 2)
        lower_crop = int(rows * self.crop_ver)
        image = image[lower_crop:, left_bound:right_bound]
        return image
    

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
            cv_image = self.rosimg_cv_bridge.imgmsg_to_cv2(
                ros_image
            )
            # passthrough
            # grayscale_cv_image = grayscale_cv_image.copy()
            if len(cv_image.shape) < 3:
                cv_image = cv.cvtColor(cv_image, cv.COLOR_GRAY2BGR)
            self.process_image(cv_image)
        except CvBridgeError as e:
            rospy.logerr(f"birdseye_lane_detector:116 - CvBridge Error: {e}")
            self.offset_message.angular.z = 3
            self.center_offset_publisher.publish(self.offset_message)
            return

        rows, cols, ch = cv_image.shape
        image_midpoint_x = cols // 2
        image_midpoint_y = rows // 2

        # FIX: Literally need to implement all of this
    def process_image(self, cv_image):
        # get image shape
        (rows, cols, _) = cv_image.shape
        
        # crop image
        crop_bound = int(self.crop_orig * rows)
        cv_image = cv_image[crop_bound : rows, : cols] 
        cv.imshow("Original", cv_image)    
        
        #change perspective
        cv_image = self.change_perspective(cv_image)
        cv.imshow("Bird Eye", cv_image)
        
        # blur
        blur = cv.medianBlur(cv_image, 5)
        
        # gray image
        # gray_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        gray_image = blur.copy()
        
        #thresh
        ret, thresh_basic = cv.threshold(gray_image, # input image
                                    self.thresh,
                                    255,    # max value in image
                                    cv.THRESH_BINARY) 
        #blur        
        thresh_basic = cv.medianBlur(thresh_basic, 5)
        
        #canny
        img_canny = cv.Canny(thresh_basic, 50, 150, 3)
        
        img_canny = self.houghs(img_canny)
        
        cv.imshow("Canny", img_canny)
        
        cv_image = self.crop_image(cv_image)
        
        image = self.crop_image(img_canny)
        
        # compute coefficient and publish velocity
        self.dbscan_process(cv_image, image)
        
        cv.waitKey(3)
    
    def houghs(self, image):
    
        lines = cv.HoughLinesP(image, 3, np.pi / 180, threshold=20, minLineLength=10, maxLineGap=10)
        
        # empty image to draw filtered lines
        line_image = np.zeros_like(image)

        # filter out lines with a small slope
        if lines is not None:
            for line in lines:

                x1, y1, x2, y2 = line[0]
                
                # Calculate length of the line segment
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                
                # Calculate slope of the line
                if x2 - x1 == 0:
                    slope = float('inf')
                else:
                    slope = (y2 - y1) / (x2 - x1)
                
                # Extend the line if its length is shorter than 40 pixels and slope is >= 0.4 or <= -0.4
                if length < 1000 and abs(slope) >= 0.5: # 50, 0.5
                    # Extend the line by a factor of 1.5 times its original length
                    extend_factor = 2
                    x1_extended = int(x1 - (x2 - x1) * (extend_factor - 1))
                    y1_extended = int(y1 - (y2 - y1) * (extend_factor - 1))
                    x2_extended = int(x2 + (x2 - x1) * (extend_factor - 1))
                    y2_extended = int(y2 + (y2 - y1) * (extend_factor - 1))
                    
                    cv.line(line_image, (x1_extended, y1_extended), (x2_extended, y2_extended), (255, 255, 255), 2)
                elif abs(slope) >= 0.5:
                    cv.line(line_image, (x1, y1), (x2, y2), (255, 255, 255), 2)
        return line_image
    
    def dbscan_process(self, image, line_image):
        
        rows, cols, ch = image.shape
        # get white points
        points = np.column_stack(np.where(line_image > 0))   
        
        if len(points) == 0:
            print("No white pixels detected.")
            self.publish_angle(None, None, 2)
            return

        # downsample the points uniformly for clustering
        # downsample_factor = 10
        # points = points[::downsample_factor]
        downsample_factor = 100
        points = points[::downsample_factor]

        # perform density based clustering
        dbscan = DBSCAN(eps=70, min_samples=3) # 100
        clusters = dbscan.fit_predict(points)

        # get the two clusters closest to bottom with certain minimum size
        unique_labels, counts = np.unique(clusters, return_counts=True)
        valid_clusters = []

        for label in unique_labels:
            if label == -1:  
                continue

            cluster_points = points[clusters == label]

            if len(cluster_points) >= 10:
                centroid_y = np.max(cluster_points[:, 0])
                valid_clusters.append((label, centroid_y))

        sorted_clusters = sorted(valid_clusters, key=lambda x: x[1], reverse=True)[:2]
        largest_labels = [sorted_clusters[i][0] for i in range(min(2, len(sorted_clusters)))]
        colors = [(0, 255, 0), (0, 0, 255)]
        label_to_color = {label: colors[i] for i, label in enumerate(largest_labels)}

        # conditions
        if len(largest_labels) == 0:
            self.publish_angle(0, 0.5, 2)
            return
        
        # if only one cluster
        elif len(largest_labels) == 1:
            lane_points = points[clusters == largest_labels[0]]
            centroid_x = lane_points[np.argmax(lane_points[:, 0])][1]
            image2 = image.copy()
            for p in lane_points:
                cv.circle(image2, (int(p[1]), int(p[0])), 5, (0,255,0), 2)
            cv.imshow("Labeled points", image2)
            cv.waitKey(3) 
            cy = rows // 2
            if centroid_x < (cols // 2):
                cx = cols
                self.publish_angle(1, 0.5, 0)

            else:
                cx = 0
                self.publish_angle(-1, 0.5, 0)
        else:
            # if two clusters, find center of both
            lane_centroids_x = []
            lane_centroids_y = []
            image2 = image.copy()
            for label in largest_labels:

                lane_points = points[clusters == label]
                lane_points = lane_points[np.argsort(lane_points[:, 1])]

                coefficients = np.polyfit(lane_points[:, 0], lane_points[:, 1], deg=1)
                polynomial = np.poly1d(coefficients)

                x_values = np.linspace(0, rows, num=100)
                y_values = polynomial(x_values).astype(int)
                
                for p in lane_points:
                    cv.circle(image2, (int(p[1]), int(p[0])), 5, label_to_color[label], 2)
                for i in range(len(y_values) - 1):
                    cv.line(image, (int(y_values[i]), int(x_values[i])), 
                            (int(y_values[i+1]), int(x_values[i+1])), label_to_color[label], 5)
                
                centroid_x = np.mean(lane_points[:, 1]) 
                centroid_y = np.mean(lane_points[:, 0]) 
                lane_centroids_x.append(centroid_x)
                lane_centroids_y.append(centroid_y)

            # get cx from the average of the two clusters' centroids
            cx = int(np.mean(lane_centroids_x))
            cy = int(np.mean(lane_centroids_y))
            self.publish_angle((2*cx-cols) / cols, float(cy) / rows, 0)

        cv.arrowedLine(image, (cols//2,rows-1), (cx,cy), (255, 255, 255), 2)
        cv.imshow("Polynomial lanes", image)
        cv.imshow("Labeled points", image2)
        cv.waitKey(3)         
        
    def publish_angle(self, coeff_x, coeff_y, missed_lanes):
        
        if (coeff_x > 1):
            self.offset_message.angular.x = 1
        elif (coeff_x < -1):
            self.offset_message.angular.x = -1
        elif (abs(coeff_x) < 0.15):
            self.offset_message.angular.x = 0
        else:
            self.offset_message.angular.x = coeff_x
        self.offset_message.angular.y = coeff_y
        self.offset_message.angular.z = missed_lanes
        
        self.center_offset_publisher.publish(self.offset_message)


if __name__ == "__main__":
    try:
        detector = BirdseyeLaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
