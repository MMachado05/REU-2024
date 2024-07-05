#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from follow_lane_pkg.cfg import FollowLaneConfig 
from geometry_msgs.msg import Twist

vel_msg = Twist()
bridge = CvBridge()

def dyn_rcfg_cb(config, level):
  global thresh_value, speed, enable_drive
  thresh_value = config.thresh
  speed = config.speed
  enable_drive = config.enable_drive
  return config

def publish_vel(coef):
   global velocity_pub
   global vel_msg
   rate = rospy.Rate(50)
   if enable_drive:
      vel_msg.linear.x = speed
      vel_msg.angular.z = coef * 1
   else:
      vel_msg.linear.x = 0
      vel_msg.angular.z = 0
   velocity_pub.publish(vel_msg)
   rate.sleep()


def image_callback(ros_image):
  global bridge
  try: #convert ros_image into an opencv-compatible imageadi
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
    print(e)
    exit(1)

  (rows,cols,channels) = cv_image.shape

  frame = cv2.medianBlur(cv_image, 5)

  hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
  thresh = cv2.inRange(hsv,np.array((0, 0, 250)), np.array((180, 255, 255)))

  contours,hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    # finding contour with maximum area and store it as best_cnt
  max_area = 0
  best_cnt = contours[0]
  for cnt in contours:
      area = cv2.contourArea(cnt)
      if area > max_area:
          max_area = area
          best_cnt = cnt

    # finding centroids of best_cnt and draw a circle there
    # https://www.geeksforgeeks.org/python-opencv-find-center-of-contour/
  M = cv2.moments(best_cnt)
  cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
  cv2.circle(cv_image,(cx,cy), 15, (0,0,255), -1) # -1: fill the circle

  publish_vel((cols/2 - cx)/(cols/2))
    

  cv2.imshow("My Image Window", cv_image)
  cv2.waitKey(3)
  
if __name__ == '__main__':
  rospy.init_node('follow_line', anonymous=True)
  imgtopic = rospy.get_param("~imgtopic_name")
  rospy.Subscriber(imgtopic, Image, image_callback)
  velocity_pub = rospy.Publisher('robot1/cmd_vel', Twist, queue_size=1)
  srv = Server(FollowLaneConfig, dyn_rcfg_cb)
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
