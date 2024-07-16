#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import cv2 as cv
import numpy as np

# TODO: Add any imports that the implementation will need

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
    from_right_offset: int
    dilation_iterations: int

    found_contours: bool

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

        self.from_right_offset = 180
        self.dilation_iterations = 3

        # Misc.
        self.rosimg_cv_bridge = CvBridge()
        self.offset_message = Twist()

        self.found_contours = True

    def _dynamic_reconfig_callback(self, config, _):
        """
        Callback function for dynamic reconfigure.

        Parameters
        ----------
        config : DBScanLaneDetectorConfig
            The new dynamic reconfigure parameters.
        """
        self.display_desired_twist_image = config.display_desired_twist_image

        self.from_right_offset = config.from_right_offset
        self.dilation_iterations = config.dilation_iterations

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
            rospy.logerr(f"largest_contour_lane_detector:109 - CvBridge Error: {e}")
            self.offset_message.angular.z = 3
            self.center_offset_publisher.publish(self.offset_message)
            return

        rows, cols = grayscale_cv_image.shape
        image_midpoint_x = cols // 2
        image_midpoint_y = rows // 2

        # Dilate image
        kernel = np.ones((5, 5), np.uint8)
        grayscale_cv_image = cv.dilate(
            grayscale_cv_image, kernel, iterations=self.dilation_iterations
        )

        right_half_image = grayscale_cv_image[:, image_midpoint_x:]
        contours, _ = cv.findContours(
            right_half_image, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            if self.found_contours:
                rospy.logerr(
                    "largest_contour_lane_detector:126 - Cannot find contours."
                )
                self.found_contours = False
            self.offset_message.angular.x = 0
            self.offset_message.angular.y = 0.5
            self.offset_message.angular.z = 1
            self.center_offset_publisher.publish(self.offset_message)
            return

        if not self.found_contours:
            rospy.loginfo("largest_contour_lane_detector:136 - Found contours.")
            self.found_contours = True

        # find the contour with the largest area
        max_area = cv.contourArea(contours[0])
        best_contour = contours[0]
        for contour in contours:
            area = cv.contourArea(contour)
            if area > max_area:
                max_area = area
                best_contour = contour

        M = cv.moments(
            best_contour
        )  # find moments of contour, only one since only one line shown on the image at a time
        try:
            contour_x, contour_y = int(M["m10"] / M["m00"]), int(
                M["m01"] / M["m00"]
            )  # find center of contour
        except ZeroDivisionError:
            rospy.logerr(
                "largest_contour_lane_detector:157 - Unexpected divide-by-zero error."
            )
            self.offset_message.angular.x = 0
            self.offset_message.angular.y = 0.5
            self.offset_message.angular.z = 3
            self.center_offset_publisher.publish(self.offset_message)
            return

        # Rescale contour to fit on uncropped version of image
        best_contour = best_contour + np.array([image_midpoint_x, 0])

        # Generate offset
        desired_midpoint_x = (
            contour_x + image_midpoint_x
        )  # Need to move the "center" into a valid point on the original image
        desired_midpoint_x -= self.from_right_offset
        desired_midpoint_y = contour_y

        if self.display_desired_twist_image:
            cv.drawContours(color_cv_image, [best_contour], -1, (0, 0, 255), 10)
            cv.circle(
                color_cv_image,
                (desired_midpoint_x, desired_midpoint_y),
                10,
                (0, 255, 0),
                -1,
            )
            cv.imshow("Desired Twist message", color_cv_image)
            cv.waitKey(1)
            # Gotta make sure to set *all* of these to 1; 0 makes everything freeeze.

        desired_twist_x = (desired_midpoint_x - image_midpoint_x) / image_midpoint_x
        desired_twist_y = (rows - desired_midpoint_y) / rows
        # NOTE: The image origin is at the top left, so we need to invert the y value

        self.offset_message.angular.x = desired_twist_x
        self.offset_message.angular.y = desired_twist_y
        self.offset_message.angular.z = 1
        self.center_offset_publisher.publish(self.offset_message)


if __name__ == "__main__":
    try:
        detector = LargestContourLaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
