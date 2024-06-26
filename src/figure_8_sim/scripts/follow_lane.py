#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from std_msgs.msg import Bool, Float64

bridge = CvBridge()
yellow_msg = Bool()
angular_vel = Float64()

cx = 0

# lane following using computer vision
# publishes angular velocity and intersection (yellow) detection

# image processing
def image_callback(ros_image):

    try: #convert ros_image into an opencv-compatible imageadi
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    # extract contours
    rows, cols, _ = cv_image.shape
    cv_image = cv_image[int(rows/2.5):, :]    
    hsv = cv.cvtColor(cv_image,cv.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    thresh = cv.inRange(hsv, lower_white, upper_white)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    cv.drawContours(cv_image, contours, -1, (0,0,255), 10)

    # find largest contour
    max_area = 0
    max_c = None
    for c in contours:
        M = cv.moments(c)
        if M['m00'] != 0:
            cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        area = cv.contourArea(c)
        if area > max_area:
            max_area = area
            max_c = c

    try:
        M = cv.moments(max_c)
        cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        cx -= 180
        cv.circle(cv_image, (cx,cy), 10, (0,0,0), -1) # -1 fill the circle
    except ZeroDivisionError or UnboundLocalError:
        cx = 0
        pass
    # cv.circle(cv_image, (cx,cy), 10, (0,0,0), -1) # -1 fill the circle
    cv.imshow('Contours with centroid dot', cv_image)
    cv.waitKey(3)
    
    # publish angular vel
    mid = cols // 2
    angular_threshold = 20 
    if mid < cx - angular_threshold:
      angular_vel = -abs((mid - cx) / mid) 
    elif mid > cx + angular_threshold:
      angular_vel = abs((mid - cx) / mid)
    else:
      angular_vel = 0
    angular_vel_pub.publish(angular_vel)

    # yellow mask
    lower_yellow = np.array([20, 100, 100]) 
    upper_yellow = np.array([30, 255, 255])
    yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)

    # check for intersection
    num_yellow_pix = cv.countNonZero(yellow_mask)
    yellow_pct = (100 * num_yellow_pix) / (rows * cols)
    if yellow_pct > 2:
        yellow_msg = Bool(data=True)
    else:
        yellow_msg = Bool(data=False)

    # publish yellow 
    yellow_pub.publish(yellow_msg)

  
# main method
if __name__ == '__main__':
    rospy.init_node('follow_lane')
    imgtopic = rospy.get_param("~imgtopic_name") 
    rospy.Subscriber(imgtopic, Image, image_callback)
    yellow_pub = rospy.Publisher('yellow', Bool, queue_size=1)
    angular_vel_pub = rospy.Publisher('angular_vel', Float64, queue_size=1)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass