#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from preprocessors_pkg.cfg import CropOnlyPreprocessorConfig  # packageName.cfg
import cv2 as cv
import numpy as np


class CropOnlyImagePreprocessor:
    """
    A ROS node for preparing images for line detection and lane
    following.

    Uses dynamic reconfigure to decide which preprocessing steps
    to run.

    Preprocessing logic taken from Beñat's "advanced_dbw_follow_lane.py" code.
    """

    crop_left: float
    crop_right: float
    crop_top: float
    crop_bottom: float

    display_preprocessed_image: bool

    raw_img_subscriber: rospy.Subscriber
    processed_img_publisher: rospy.Publisher
    dyn_rcfg_srv: Server

    rosimg_cv_bridge: CvBridge
    image_is_displaying: bool

    # ------------------------------------------------
    # ------- Internal state-related functions -------
    # ------------------------------------------------
    def __init__(self):
        """
        Initializes an image-preprocessing node.

        Sets up subscribers, publishers, and dynamic reconfigure.
        """
        # Node architecture
        rospy.init_node("crop_only_line_detection_preprocessor", anonymous=True)
        self.raw_img_subscriber = rospy.Subscriber(
            rospy.get_param("~img_in_topic"), Image, self._preprocess_image
        )
        # TODO: Add a subcscriber for live cropping
        self.processed_img_publisher = rospy.Publisher(
            rospy.get_param("~img_out_topic"), Image, queue_size=1
        )
        self.dyn_rcfg_srv = Server(CropOnlyPreprocessorConfig, self._dynamic_reconfig_callback)

        # Initially-set preprocessing parameters
        self.crop_top = 0.0
        self.crop_bottom = 1.0
        self.crop_left = 0.0
        self.crop_right = 1.0  # Crop out the top half of the image

        self.display_preprocessed_image = False

        # Misc.
        self.rosimg_cv_bridge = CvBridge()
        self.image_is_displaying = False

    def _dynamic_reconfig_callback(self, config, _):
        """
        Callback function for dynamic reconfigure.

        Parameters
        ----------
        config: PreprocessorConfig
            Configuration for preprocessing.
        """
        self.crop_top = config.crop_top / 100
        self.crop_bottom = config.crop_bottom / 100
        self.crop_left = config.crop_left / 100
        self.crop_right = config.crop_right / 100

        self.display_preprocessed_image = config.display_preprocessed_image

        return config

    # -----------------------------------------------------
    # ----------- Image Preprocessing functions -----------
    # -----------------------------------------------------
    def _preprocess_image(self, ros_image: Image) -> None:
        """
        Preprocess image based on dynamic reconfigure settings.

        Parameters
        ----------
        ros_image: Image
            ROS image to be preprocessed.
        """
        # Convert ROS image to OpenCV image
        try:
            cv_image = self.rosimg_cv_bridge.imgmsg_to_cv2(ros_image)
        except CvBridgeError as e:
            rospy.logerr(f"preprocessor - ROS to OpenCV Bridge Error: {e}")
            return
        except Exception as b:
            pass

        # Initial crop
        rows, cols, _ = cv_image.shape
        cv_image = cv_image[
            int(rows * self.crop_top) : int(rows * self.crop_bottom),
            int(cols * self.crop_left) : int(cols * self.crop_right),
        ]
        rows, cols, _ = cv_image.shape

        # Display and publish preprocessed image
        if self.display_preprocessed_image:
            cv.imshow("Preprocessed image", cv_image)
            cv.waitKey(1)

        try:
            # cv_image = cv.cvtColor(cv_image, cv.COLOR_GRAY2BGR)
            preprocessed_image = self.rosimg_cv_bridge.cv2_to_imgmsg(cv_image)
            # rospy.loginfo(f"preprocessed image encoding: {preprocessed_image.encoding}")
        except CvBridgeError as e:
            rospy.logerr(f"preprocessor - OpenCV to ROS Bridge Error: {e}")
            return

        self.processed_img_publisher.publish(preprocessed_image)

        # TODO: Figure out how to get this to work.
        if not self.display_preprocessed_image and self.image_is_displaying:
            cv.destroyAllWindows()
            self.image_is_displaying = self.display_preprocessed_image


if __name__ == "__main__":
    try:
        preprocessor = CropOnlyImagePreprocessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
