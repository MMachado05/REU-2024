#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from line_detection.cfg import PreprocessorConfig  # packageName.cfg
import cv2 as cv
import numpy as np


class ROSImagePreprocessor:
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

    use_warp_perspective: bool
    warp_topleft_x: float
    warp_topleft_y: float
    warp_bottomleft_x: float
    warp_bottomleft_y: float
    warp_topright_x: float
    warp_topright_y: float
    warp_bottomright_x: float
    warp_bottomright_y: float

    use_median_blur: bool

    filter_white: bool
    white_thresh: int

    use_canny: bool
    canny_lower_thresh: int
    canny_upper_thresh: int

    use_live_crop: bool
    live_crop_weight: float

    display_preprocessed_image: bool

    raw_img_subscriber: rospy.Subscriber
    processed_img_publisher: rospy.Publisher
    dyn_rcfg_srv: Server
    # TODO: Add required live cropping subscriber

    rosimg_cv_bridge: CvBridge

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
        self.dyn_rcfg_srv = Server(PreprocessorConfig, self._dynamic_reconfig_callback)

        # Initially-set preprocessing parameters
        self.initial_crop_top = 0.5
        self.initial_crop_bottom = 1.0
        self.initial_crop_left = 0.0
        self.initial_crop_right = 1.0  # Crop out the top half of the image

        self.use_warp_perspective = False
        self.warp_topleft_x = 0.0
        self.warp_topleft_y = 0.0
        self.warp_bottomleft_x = 0.0
        self.warp_bottomleft_y = 1.0
        self.warp_topright_x = 1.0
        self.warp_topright_y = 0.0
        self.warp_bottomright_x = 1.0
        self.warp_bottomright_y = 1.0

        self.use_poly_mask = False
        self.poly_top_left = 0.0
        self.poly_top_right = 1.0
        self.poly_bottom_left = 1.0
        self.poly_bottom_right = 1.0

        # TODO: Add live cropping

        self.filter_white = True
        self.white_thresh = 220

        self.use_median_blur = True

        self.use_canny = True
        self.canny_lower_thresh = 50
        self.canny_upper_thresh = 150

        self.use_live_crop = False
        self.live_crop_weight = 0.5

        self.display_preprocessed_image = False

        # Misc.
        self.rosimg_cv_bridge = CvBridge()

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

        self.use_warp_perspective = config.use_warp_perspective
        self.warp_topleft_x = config.topleft_x / 100
        self.warp_topleft_y = config.topleft_y / 100
        self.warp_bottomleft_x = config.bottomleft_x / 100
        self.warp_bottomleft_y = config.bottomleft_y / 100
        self.warp_topright_x = config.topright_x / 100
        self.warp_topright_y = config.topright_y / 100
        self.warp_bottomright_x = config.bottomright_x / 100
        self.warp_bottomright_y = config.bottomright_y / 100

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

        self.use_live_crop = config.use_live_crop
        self.live_crop_weight = config.live_crop_weight

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
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Initial crop
        rows, cols, _ = cv_image.shape
        cv_image = cv_image[
            int(rows * self.initial_crop_top) : int(rows * self.initial_crop_bottom),
            int(cols * self.initial_crop_left) : int(cols * self.initial_crop_right),
        ]
        rows, cols, _ = cv_image.shape

        # Perspective warping
        if self.use_warp_perspective:
            topleft = [int(cols * self.warp_topleft_x), int(rows * self.warp_topleft_y)]
            topright = [int(cols * self.warp_topright_x), int(rows * self.warp_topright_y)]
            bottomleft = [int(cols * self.warp_bottomleft_x), int(rows * self.warp_bottomleft_y)]
            bottomright = [int(cols * self.warp_bottomright_x), int(rows * self.warp_bottomright_y)]

            perspective_1 = np.array([[0, 0], [cols, 0], [cols, rows], [0, rows]], dtype=np.float32)
            perspective_2 = np.array([topleft, topright, bottomright, bottomleft], dtype=np.float32)

            matrix = cv.getPerspectiveTransform(perspective_1, perspective_2)
            cv_image = cv.warpPerspective(cv_image, matrix, (cols, rows))

        # Add poly mask
        if self.use_poly_mask:
            # NOTE: I have no idea if this works with perspective warping enabled, but I don't
            #   think they should be used together, anyways
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

        # TODO: Add live cropping

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
            self.processed_img_publisher.publish(
                self.rosimg_cv_bridge.cv2_to_imgmsg(cv_image)
            )
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")


if __name__ == "__main__":
    try:
        ROSImagePreprocessor()
    except rospy.ROSInterruptException:
        pass
