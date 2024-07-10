#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import cv2 as cv

# TODO: Add any imports that the implementation will need

from dynamic_reconfigure.server import Server
from lane_detectors_pkg.cfg import BirdseyeLaneDetectorConfig  # packageName.cfg


class BirdseyeLaneDetector:
    """
    K-means lane detector node.

    Uses Beñat's "kmeans_lane_follow.py" code.
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
        rospy.init_node("kmeans_lane_detector", anonymous=True)

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

        # FIX: Literally need to implement all of this


if __name__ == "__main__":
    try:
        detector = BirdseyeLaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
