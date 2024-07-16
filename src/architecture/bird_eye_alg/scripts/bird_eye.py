#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from bird_eye_alg.cfg import BirdEyeConfig
from geometry_msgs.msg import Twist


class BirdEye:
  
    def __init__(self, cam_top, twist_top):
        self.config = None
        
        self.vel_msg = Twist()
        self.bridge = CvBridge()
        
        self.velocity_pub = rospy.Publisher(twist_top, Twist, queue_size=1)
        self.sub = rospy.Subscriber(cam_top, Image, self.image_callback)
        self.srv = Server(BirdEyeConfig, self.dyn_rcfg_cb)
    
    def dyn_rcfg_cb(self, config, level):
        self.config = config
        return config

    def image_callback(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            self.process_image(cv_image)
        except CvBridgeError as e:
            print(e)
    
    def change_perspective(self, img):
        rows, cols, _ = img.shape
        
        p1 = [0, 0]
        p2 = [cols//2, 0]
        p3 = [cols//2 - int(cols * self.config.pers), rows]
        p4 = [int(cols * self.config.pers), rows]
        
        per1 = np.float32([[0, 0], [cols, 0], [cols, rows], [0, rows]])
        per2 = np.float32([p1, p2, p3, p4])
        
        matrix = cv2.getPerspectiveTransform(per1, per2)
        result = cv2.warpPerspective(img, matrix, (cols//2, rows))
        return result
    
    def compute_y(self, m, c ,x):
        return int(m * x + c)
    
    def compute_center(self, image):
        # crop image
        rows, cols = image.shape
        left_bound = cols//2 - int(cols*self.config.crop_lines/2)
        right_bound = cols//2 + int(cols*self.config.crop_lines/2)
        image = image[rows*2//3:, left_bound:right_bound]
        r, c = image.shape
        
        # get all white points (edge lane points)
        lane_points = cv2.findNonZero(image)
        
        # get coordinates of all left and right lane points
        left_x = []
        left_y = []
        right_x = []
        right_y = []
        if lane_points is not None:
            for point in lane_points:
                pnt = point[0]
                if pnt[0] < c//2:
                    left_y.append(pnt[0])
                    left_x.append(pnt[1])
                else:
                    right_y.append(pnt[0])
                    right_x.append(pnt[1])
        else:
            return 0
        
        # get coefficients for lsrl for left lane
        try:
            A = np.vstack([left_x, np.ones(len(left_x))]).T
            m_left, c_left = np.linalg.lstsq(A, left_y)[0]
        except:
            m_left = 0
            c_left = 0
        # draw left line
        cv2.line(image, (self.compute_y(m_left, c_left, 0), 0), (self.compute_y(m_left, c_left, r), r), 120, 3)
        
        # get coefficients for lsrl for right lane
        try:
            A = np.vstack([right_x, np.ones(len(right_x))]).T
            m_right, c_right = np.linalg.lstsq(A, right_y)[0]
        except:
            m_right = 0
            c_right = c
        #draw right line
        cv2.line(image, (self.compute_y(m_right, c_right, 0), 0), (self.compute_y(m_right, c_right, r), r), 170, 3)
        
        # compute midpoint of two lanes
        y_left = self.compute_y(m_left, c_left, r//2)
        if y_left > c//2:
            y_left = 0
        
        y_right = self.compute_y(m_right, c_right, r//2)
        if y_right < c//2:
            y_right = c
        
        cx = int((y_left+y_right)//2)

        # draw, show & return result
        cv2.circle(image, (cx, r//2), 3, 255, -1)
        
        cv2.imshow("Lines", image)
        cv2.waitKey(3)
        
        return ( (c/2-cx) / c/2) * self.config.coef_mult  


    def process_image(self, cv_image):
        # get image shape
        (rows, cols, channels) = cv_image.shape
        
        # crop image
        crop_bound = int(self.config.crop_orig * rows)
        cv_image = cv_image[crop_bound : rows, : cols] 
        cv2.imshow("Original", cv_image)    
        
        #change perspective
        cv_image = self.change_perspective(cv_image)
        cv2.imshow("Bird Eye", cv_image)
        
        # blur
        blur = cv2.medianBlur(cv_image, 5)
        
        # gray image
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        #thresh
        ret, thresh_basic = cv2.threshold(gray_image, # input image
                                    self.config.thresh,
                                    255,    # max value in image
                                    cv2.THRESH_BINARY) 
        #blur        
        thresh_basic = cv2.medianBlur(thresh_basic, 5)
        
        #canny
        img_canny = cv2.Canny(thresh_basic, 
                              self.config.canny_thresh1, 
                              self.config.canny_thresh2)
        cv2.imshow("Canny", img_canny)
        
        # compute coefficient and publish velocity
        coef = self.compute_center(img_canny)
        self.publish_vel(coef)

    def publish_vel(self, coef):
        ang_speed = self.config.ang_vel
        if self.config.enable_drive:
            self.vel_msg.linear.x = self.config.speed * (1.1 - abs(coef))
            if (coef + self.config.tolerance < 0):
                self.vel_msg.angular.z = coef * ang_speed
            elif (coef - self.config.tolerance > 0):
                self.vel_msg.angular.z = coef * ang_speed
            else:
                self.vel_msg.angular.z = 0
        else:
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
        self.velocity_pub.publish(self.vel_msg)

if __name__ == '__main__':
    rospy.init_node("bird", anonymous=True)
    cam_top = "camera/image_raw"
    robot_vel = "cmd_vel"
    BirdEye(cam_top, robot_vel)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass