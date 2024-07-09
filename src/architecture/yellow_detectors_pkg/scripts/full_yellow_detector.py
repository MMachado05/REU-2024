#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from yellow_detectors_pkg.cfg import FullYellowDetectorConfig  # packageName.cfg

import cv2 as cv
import numpy as np

class FullYellowDetector:
    """
    A simple yellow-pixel detector. The only configurable aspect of this node is 
    the percentage threshold of how much yellow is in the received image.
    """

    image_subscriber: rospy.Subscriber
    yellow_publisher: rospy.Publisher

    display_yellow_mask: bool

    yellow_proportion_thresh: float

    hue_low: int
    hue_high: int
    sat_low: int
    sat_high: int
    val_low: int
    val_high: int

    def __init__(self):
        """
        Initializes a yellow detector node.
        """
        rospy.init_node("full_yellow_detector", anonymous=True)
        self.image_subscriber = rospy.Subscriber(
            rospy.get_param("~img_in_topic"), Image, self._image_callback
        )
        self.yellow_publisher = rospy.Publisher(
            rospy.get_param("~decision_out_topic"), Bool, queue_size=1
        )
        self.dyn_rcfg_srv = Server(
            FullYellowDetectorConfig, self._dynamic_reconfig_callback
        )

        self.display_yellow_mask = False

        self.yellow_proportion_thresh = 0.02

        self.hue_low = 0
        self.hue_high = 24
        self.sat_low = 27
        self.sat_high = 255
        self.val_low = 70
        self.val_high = 255

        self.rosimg_cv_bridge = CvBridge()

        rospy.spin()

    def _dynamic_reconfig_callback(self, config, _):
        """
        Callback function for dynamic reconfigure.

        Parameters
        ----------
        config: FullYellowDetectorConfig
            Configuration for yellow detector.
        """
        self.display_yellow_mask = config.display_yellow_mask

        self.yellow_percentage_thresh = config.yellow_percentage_thresh

        self.hue_low = config.hue_low
        self.hue_high = config.hue_high
        self.sat_low = config.sat_low
        self.sat_high = config.sat_high
        self.val_low = config.val_low
        self.val_high = config.val_high

    def _image_callback(self, ros_image: Image) -> None:
        """
        Callback function for image subscriber.

        Parameters
        ----------
        ros_image: Image
            ROS image to be processed.
        """
        try:
            cv_image = self.rosimg_cv_bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"full_yellow_detector:94 - ROS to OpenCV Bridge Error: {e}")
            return

        hsv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        rows, cols, _ = cv_image.shape

        lower_yellow = np.array([self.hue_low, self.sat_low, self.val_low]) 
        upper_yellow = np.array([self.hue_high, self.sat_high, self.val_high]) 

        yellow_mask = cv.inRange(hsv_image, lower_yellow, upper_yellow)

        if self.display_yellow_mask:
            cv.imshow("Yellow pixels", yellow_mask)
            cv.waitKey(1)

        num_yellow_px = cv.countNonZero(yellow_mask)

        if num_yellow_px / (rows * cols) > self.yellow_proportion_thresh:
            self.yellow_publisher.publish(Bool(data=True))
        else:
            self.yellow_publisher.publish(Bool(data=False))

if __name__ == "__main__":
    try:
        FullYellowDetector()
    except rospy.ROSInterruptException:
        pass
