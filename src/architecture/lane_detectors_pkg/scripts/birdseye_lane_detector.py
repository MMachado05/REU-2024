#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import cv2 as cv
import numpy as np
import math

# TODO: Add any imports that the implementation will need

from dynamic_reconfigure.server import Server
from lane_detectors_pkg.cfg import BirdseyeLaneDetectorConfig  # packageName.cfg


class BirdseyeLaneDetector:
    """
    Birds Eye lane detector node.

    Uses "bird_eye_houghs.py" code.
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
        
        # other
        self.mask = None
        self.midpoint = None

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
        
        # compute coefficient and publish velocity
        self.compute_center(img_canny)
    
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
                if length < 100 and abs(slope) >= 1.5: # 50, 0.5
                    # Extend the line by a factor of 1.5 times its original length
                    extend_factor = 2
                    x1_extended = int(x1 - (x2 - x1) * (extend_factor - 1))
                    y1_extended = int(y1 - (y2 - y1) * (extend_factor - 1))
                    x2_extended = int(x2 + (x2 - x1) * (extend_factor - 1))
                    y2_extended = int(y2 + (y2 - y1) * (extend_factor - 1))
                    
                    cv.line(line_image, (x1_extended, y1_extended), (x2_extended, y2_extended), (255, 255, 255), 2)
                elif abs(slope) >= 1.5:
                    cv.line(line_image, (x1, y1), (x2, y2), (255, 255, 255), 2)
        return line_image
    
    def compute_y(self, m, c ,x):
        return int(m * x + c)

    def compute_center(self, image):
        # crop image
        rows, cols = image.shape
        left_bound = cols//2 - int(cols * self.crop_hor / 2)
        right_bound = cols//2 + int(cols * self.crop_hor / 2)
        lower_crop = int(rows * self.crop_ver)
        image = image[lower_crop:, left_bound:right_bound]
        r, c = image.shape
        
        # create mask
        if self.mask is None or self.mask.shape != image.shape:
            self.mask = np.zeros((r, c), np.uint8)
            self.mask.fill(255)
        if self.midpoint == None:
            self.midpoint = c // 2
        
            
        image = cv.bitwise_and(image, self.mask)
        
        # get all white points (edge lane points)
        lane_points = cv.findNonZero(image)
        
        # get coordinates of all left and right lane points
        left_x = []
        left_y = []
        right_x = []
        right_y = []
        if lane_points is not None:
            for point in lane_points:
                pnt = point[0]
                if pnt[0] < self.midpoint:
                    left_y.append(pnt[0])
                    left_x.append(pnt[1])
                else:
                    right_y.append(pnt[0])
                    right_x.append(pnt[1])
        else:
            self.publish_angle(None, None, 2)
            return 
        
        # get coefficients for lsrl for left lane
        if (len(left_x)>=10 and len(left_y)>=10):
            A = np.vstack([left_x, np.ones(len(left_x))]).T
            m_left, c_left = np.linalg.lstsq(A, left_y)[0]
        else:
            m_left = 0
            c_left = 0
        
        
        # get coefficients for lsrl for right lane
        if (len(right_x)>=10 and len(right_y)>=10):
            A = np.vstack([right_x, np.ones(len(right_x))]).T
            m_right, c_right = np.linalg.lstsq(A, right_y)[0]
        else:
            m_right = 0
            c_right = c
        
        self.draw_mask((m_left+m_right)/2, image)
        
        image = cv.cvtColor(image, cv.COLOR_GRAY2BGR)
        
        # compute midpoint of two lanes
        y_left = self.compute_y(m_left, c_left, r//2)
        if y_left > self.midpoint:
            y_left = 0
        else:
            cv.line(image, (self.compute_y(m_left, c_left, 0), 0), (self.compute_y(m_left, c_left, r), r), (0, 0, 255), 3)
        
        y_right = self.compute_y(m_right, c_right, r//2)
        if y_right < self.midpoint:
            y_right = c
        else:
            cv.line(image, (self.compute_y(m_right, c_right, 0), 0), (self.compute_y(m_right, c_right, r), r), (0, 255, 0), 3)
        
        cx = int((y_left+y_right)//2)
        cy = r // 2
        mid = c // 2
        
        angle = float(math.degrees(math.atan2(abs(mid - cx), abs(rows - cy))) / 2) * 50 / 180
        
        if cx < mid:
            self.publish_angle(-angle, 0.5, 0)
        else:
            self.publish_angle(angle, 0.5, 0)

        # draw, show & return result
        cv.circle(image, (cx, cy), 3, (255, 255, 255), -1)    
        cv.line(image, (self.midpoint, 0),(self.midpoint, r), (255, 0, 0), 3)  
        cv.imshow("Lines", image)
        cv.waitKey(3)
    
    def draw_mask(self, av_m, image):
        rows, cols = image.shape
        mask = np.zeros((rows, cols), np.uint8)
        mask.fill(255)
        if (av_m < 0):
            p1 = (0, rows//2)
            p2 = (int(-rows//2*av_m), 0)
            p3 = (0, 0)
            fig = [p1, p2, p3]
            cv.fillPoly(mask, pts=[np.array(fig)], color=0)
            self.midpoint = int(cols//2 + int(-rows//3*av_m))
        else:
            p1 = (cols, rows//2)
            p2 = (int(cols-rows//2*av_m), 0)
            p3 = (cols, 0)
            fig = [p1, p2, p3]
            cv.fillPoly(mask, pts=[np.array(fig)], color=0)
            self.midpoint = int(cols//2 - rows//3*av_m)
        
        cv.imshow("mask", mask)
        cv.waitKey(3)
        self.mask = mask
        
    def publish_angle(self, coeff_x, coeff_y, missed_lanes):
        
        if missed_lanes == 2:
            self.offset_message.angular.z = 2
        elif missed_lanes == 1:
            self.offset_message.angular.z = 1
        else:
            if (coeff_x > 1):
                self.offset_message.angular.x = 1
            elif (coeff_x < -1):
                self.offset_message.angular.x = -1
            elif (abs(coeff_x) < 0.15):
                self.offset_message.angular.x = 0
            else:
                self.offset_message.angular.x = coeff_x
            self.offset_message.angular.y = coeff_y
            self.offset_message.angular.z = 0
        
        self.center_offset_publisher.publish(self.offset_message)


if __name__ == "__main__":
    try:
        detector = BirdseyeLaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
