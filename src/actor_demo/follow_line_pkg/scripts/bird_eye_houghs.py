#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Float64
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from follow_line_pkg.cfg import FollowLineConfig  # packageName.cfg
from geometry_msgs.msg import Twist
import cv2 as cv
import numpy as np
from sklearn.cluster import DBSCAN
import time
import math

 
config_var = None
pub_angle = None
bridge = CvBridge()
    
def dyn_rcfg_cb(config, level):
    global config_var
    config_var = config
    return config

def image_callback(ros_image):
    global bridge
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        process_image(cv_image)
    except CvBridgeError as e:
        print(e)

def change_perspective(img):
    global config_var
    rows, cols, _ = img.shape
    
    p1 = [0, 0]
    p2 = [cols//2, 0]
    p3 = [cols//2 - int(cols * config_var.bird1), rows]
    p4 = [int(cols * config_var.bird1), rows]
    
    per1 = np.float32([[0, 0], [cols, 0], [cols, rows], [0, rows]])
    per2 = np.float32([p1, p2, p3, p4])
    
    matrix = cv.getPerspectiveTransform(per1, per2)
    result = cv.warpPerspective(img, matrix, (cols//2, rows))
    return result

def houghs(image):
    
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
            if length < 100 and abs(slope) >= 0.5: # 50, 0.5
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
    
    

def compute_y(m, c ,x):
    return int(m * x + c)

def compute_center(image):
    global config_var
    # crop image
    rows, cols = image.shape
    left_bound = cols//2 - int(cols/4)
    right_bound = cols//2 + int(cols/4)
    image = image[rows//2:, left_bound:right_bound]
    r, c = image.shape
    
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
            if pnt[0] < c//2:
                left_y.append(pnt[0])
                left_x.append(pnt[1])
            else:
                right_y.append(pnt[0])
                right_x.append(pnt[1])
    else:
        print("Errrorrororor")
        pub_angle.publish(0.0)
        return 
    
    # get coefficients for lsrl for left lane
    try:
        A = np.vstack([left_x, np.ones(len(left_x))]).T
        m_left, c_left = np.linalg.lstsq(A, left_y)[0]
    except:
        m_left = 0
        c_left = 0
    # draw left line
    cv.line(image, (compute_y(m_left, c_left, 0), 0), (compute_y(m_left, c_left, r), r), 120, 3)
    
    # get coefficients for lsrl for right lane
    try:
        A = np.vstack([right_x, np.ones(len(right_x))]).T
        m_right, c_right = np.linalg.lstsq(A, right_y)[0]
    except:
        m_right = 0
        c_right = c
    #draw right line
    cv.line(image, (compute_y(m_right, c_right, 0), 0), (compute_y(m_right, c_right, r), r), 170, 3)
    
    # compute midpoint of two lanes
    y_left = compute_y(m_left, c_left, r//2)
    if y_left > c//2:
        y_left = 0
    
    y_right = compute_y(m_right, c_right, r//2)
    if y_right < c//2:
        y_right = c
    
    cx = int((y_left+y_right)//2)
    cy = r // 2
    mid = c // 2
    
    angle = float(math.degrees(math.atan2(abs(mid - cx), abs(rows - cy))) / 2) * 5

    # draw, show & return result
    cv.circle(image, (cx, cy), 3, 255, -1)    
    cv.imshow("Lines", image)
    cv.waitKey(3)
    
    if cx:
  
        angular_threshold = 10
        if mid < cx - angular_threshold:
            pub_angle.publish(-angle)
            print(-angle)
        elif mid > cx + angular_threshold:
            print(angle)
            pub_angle.publish(angle)
        else:
            print(0)
            pub_angle.publish(0.0)
    

def process_image(cv_image):
    global pub_angle, config_var
    # get image shape
    (rows, cols, channels) = cv_image.shape
    
    # crop image
    crop_bound = int(0.5 * rows)
    cv_image = cv_image[crop_bound : rows, : cols] 
    cv.imshow("Original", cv_image)    
    
    #change perspective
    cv_image = change_perspective(cv_image)
    cv.imshow("Bird Eye", cv_image)
    
    # blur
    blur = cv.medianBlur(cv_image, 5)
    
    # gray image
    gray_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
    
    #thresh
    ret, thresh_basic = cv.threshold(gray_image, # input image
                                config_var.thresh,
                                255,    # max value in image
                                cv.THRESH_BINARY) 
    #blur        
    thresh_basic = cv.medianBlur(thresh_basic, 5)
    
    #canny
    img_canny = cv.Canny(thresh_basic, 50, 150, 3)
    
    img_canny = houghs(img_canny)
    
    cv.imshow("Canny", img_canny)
    
    # compute coefficient and publish velocity
    compute_center(img_canny)


if __name__ == '__main__':
  rospy.init_node('advanced_dbw_follow_lane', anonymous=True) # initialize node
  imgtopic = rospy.get_param("~imgtopic_name") # private name
  rospy.Subscriber(imgtopic, Image, image_callback) # subscribe to image (so that image_callback can be called every time an image is published)
  
  pub_angle = rospy.Publisher("angle", Float64, queue_size=1)
  
  srv = Server(FollowLineConfig, dyn_rcfg_cb) # create dynamic reconfigure server that calls dyn_rcfg_cb function every time a parameter is changed
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass