#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from preprocessors_pkg.cfg import EasyBirdseyePreprocessorConfig  # packageName.cfg
import cv2 as cv
import numpy as np


class EasyBirdseyeImagePreprocessor:
    """
    A ROS node for preparing images for line detection and lane
    following.

    Uses dynamic reconfigure to decide which preprocessing steps
    to run.

    Preprocessing logic taken from Beñat's "advanced_dbw_follow_lane.py" code.
    """

    initial_crop_left: float
    initial_crop_right: float
    initial_crop_top: float
    initial_crop_bottom: float

    use_poly_mask: bool
    poly_top_left: float
    poly_top_right: float
    poly_bottom_left: float
    poly_bottom_right: float

    use_birdseye: bool
    squeeze_factor_bottom: float
    squeeze_factor_top: float

    use_median_blur: bool

    filter_white: bool
    white_thresh: int

    use_canny: bool
    canny_lower_thresh: int
    canny_upper_thresh: int

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
        rospy.init_node("line_detection_preprocessor", anonymous=True)
        self.raw_img_subscriber = rospy.Subscriber(
            rospy.get_param("~img_in_topic"), Image, self._preprocess_image
        )
        # TODO: Add a subcscriber for live cropping
        self.processed_img_publisher = rospy.Publisher(
            rospy.get_param("~img_out_topic"), Image, queue_size=1
        )
        self.dyn_rcfg_srv = Server(EasyBirdseyePreprocessorConfig, self._dynamic_reconfig_callback)

        # Initially-set preprocessing parameters
        self.initial_crop_top = 0.0
        self.initial_crop_bottom = 1.0
        self.initial_crop_left = 0.0
        self.initial_crop_right = 1.0  # Crop out the top half of the image

        self.use_birdseye = False
        self.squeeze_factor_bottom = 0.8
        self.squeeze_factor_top = 0.2

        self.use_poly_mask = False
        self.poly_top_left = 0.0
        self.poly_top_right = 1.0
        self.poly_bottom_left = 1.0
        self.poly_bottom_right = 1.0

        self.filter_white = True
        self.white_thresh = 220

        self.use_median_blur = True

        self.use_canny = True
        self.canny_lower_thresh = 50
        self.canny_upper_thresh = 150

        self.display_preprocessed_image = False

        # Misc.
        self.rosimg_cv_bridge = CvBridge()
        self.image_is_displaying = False

        # Begin preprocessing
        rospy.spin()

    def _dynamic_reconfig_callback(self, config, _):
        """
        Callback function for dynamic reconfigure.

        Parameters
        ----------
        config: PreprocessorConfig
            Configuration for preprocessing.
        """
        self.initial_crop_top = config.initial_crop_top / 100
        self.initial_crop_bottom = config.initial_crop_bottom / 100
        self.initial_crop_left = config.initial_crop_left / 100
        self.initial_crop_right = config.initial_crop_right / 100

        self.use_birdseye = config.use_birdseye
        self.squeeze_factor_bottom = config.squeeze_factor_bottom
        self.squeeze_factor_top = config.squeeze_factor_top

        self.use_poly_mask = config.use_poly_mask
        self.poly_top_left = (
            config.top_left / 100
            if config.top_left < config.top_right
            else config.top_right / 100
        )
        self.poly_top_right = (
            config.top_right / 100
            if config.top_right > config.top_left
            else config.top_left / 100
        )  # Don't allow left to go further than right, and vise versa
        self.poly_bottom_left = config.bottom_left / 100
        self.poly_bottom_right = config.bottom_right / 100

        self.use_median_blur = config.use_median_blur

        self.filter_white = config.filter_white
        self.white_thresh = config.white_thresh

        self.use_canny = config.use_canny
        self.canny_lower_thresh = config.canny_lower_thresh
        self.canny_upper_thresh = config.canny_upper_thresh

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
            cv_image = self.rosimg_cv_bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"preprocessor - ROS to OpenCV Bridge Error: {e}")
            return

        # Initial crop
        rows, cols, _ = cv_image.shape
        cv_image = cv_image[
            int(rows * self.initial_crop_top) : int(rows * self.initial_crop_bottom),
            int(cols * self.initial_crop_left) : int(cols * self.initial_crop_right),
        ]
        rows, cols, _ = cv_image.shape

        # Perspective warping
        if self.use_birdseye:
            half_width = cols / 2
            top_offset_from_edge = int(half_width * self.squeeze_factor_top)
            bottom_offset_from_edge = int(half_width * self.squeeze_factor_bottom)

            topleft = [top_offset_from_edge, 0]
            topright = [cols - top_offset_from_edge, 0]
            bottomright = [cols - bottom_offset_from_edge, rows]
            bottomleft = [bottom_offset_from_edge, rows]

            perspective_1 = np.array(
                [[0, 0], [cols, 0], [cols, rows], [0, rows]], dtype=np.float32
            )
            perspective_2 = np.array(
                [topleft, topright, bottomright, bottomleft], dtype=np.float32
            )

            matrix = cv.getPerspectiveTransform(perspective_1, perspective_2)
            cv_image = cv.warpPerspective(cv_image, matrix, (cols, rows))

        # Add poly mask
        if self.use_poly_mask:
            height, width = rows, cols
            mask = np.zeros((rows, cols), dtype="uint8")
            roi_points = [
                (int(width * self.poly_top_left), 0),
                (0, int(height * self.poly_bottom_left)),
                (0, height),
                (width, height),
                (width, int(height * self.poly_bottom_right)),
                (int(width * self.poly_top_right), 0),
            ]
            cv.fillPoly(mask, [np.array(roi_points)], (255, 255, 255))
            cv_image = cv.bitwise_and(cv_image, cv_image, mask=mask)

        if self.use_median_blur:
            cv_image = cv.medianBlur(cv_image, 5)

        if self.filter_white:
            cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
            _, cv_image = cv.threshold(
                cv_image, self.white_thresh, 255, cv.THRESH_BINARY
            )

        if self.use_canny:
            cv_image = cv.Canny(
                cv_image, self.canny_lower_thresh, self.canny_upper_thresh
            )

        # Display and publish preprocessed image
        if self.display_preprocessed_image:
            cv.imshow("Preprocessed image", cv_image)
            cv.waitKey(1)

        try:
            # cv_image = cv.cvtColor(cv_image, cv.COLOR_GRAY2BGR)
            preprocessed_image = self.rosimg_cv_bridge.cv2_to_imgmsg(cv_image, "passthrough")
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
        EasyBirdseyeImagePreprocessor()
    except rospy.ROSInterruptException:
        pass
