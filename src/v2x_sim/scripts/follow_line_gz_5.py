#!/usr/bin/env python3
# Line Following

import rospy
import cv2
from std_msgs.msg import Bool
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from v2x_sim.cfg import FollowLaneConfig   # packageName.cfg
from geometry_msgs.msg import Twist

vel_msg = Twist()
bridge = CvBridge()
red_light = False;

def redL_cb(r):
	global red_light
	if(r.data == True): # stop
		red_light = True;
	else:
		red_light = False;

def dyn_rcfg_cb(config, level):
  global thresh, drive, speed
  thresh = config.thresh
  drive = config.enable_drive
  speed = config.speed
  return config

def image_callback(ros_image):
  global bridge
  try: #convert ros_image into an opencv-compatible imageadi
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
    print(e)
  
  # from now on, you can work exactly like with opencv
  cv_image = cv2.resize(cv_image, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_AREA)
  (rows,cols,channels) = cv_image.shape

  gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
  ret, bw_image = cv2.threshold(gray_image, # input image
                                thresh,     # threshol_value,
                                255,        # max value in image
                                cv2.THRESH_BINARY) # threshold type

  # ===== Line Following Code Below ======
  bw_image = cv2.medianBlur(bw_image, 5)
  contours,hierarchy = cv2.findContours(bw_image,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

  # finding contour with maximum area and store it as best_cnt
  max_area = 0
  best_cnt = 0
  for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            best_cnt = cnt

  # finding centroids of best_cnt and draw a circle there
  M = cv2.moments(best_cnt)

  try:
        cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

        cx += 120   # Comment out for task 2, use for task 3

        cv2.circle(bw_image,(cx,cy), 15, (100, 100, 100), -1) # -1: fill the circle

        # Show it
        frame = cv2.resize(bw_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        cv2.imshow('Lane with centroid dot', frame)
  except:
        pass
  
  
  if (drive): # drive enabled
    vel_msg.linear.x = speed
    if red_light == True:
      vel_msg.linear.x = 0
    mid_view_point = cols / 2
    kz = (np.abs((cx - mid_view_point)) / mid_view_point)
    print(kz)
    if (kz >= 0.249):
      kz=0.249
      if (vel_msg.linear.x <= 2.5):
        vel_msg.linear.x = speed / 2
        cx -= 30
      if (vel_msg.linear.x >= 2.5):
        vel_msg.linear.x = speed / 3
        cx -= 40

    tolerance = 20 # Tolerance of 80 for task 2, Tolerance of 10 for task 3

    if (cx > (mid_view_point + tolerance)):
      vel_msg.angular.z = -kz
      velocity_pub.publish(vel_msg)

    elif (cx < (mid_view_point - tolerance)):
      vel_msg.angular.z = kz
      velocity_pub.publish(vel_msg)

    else:
       vel_msg.angular.z = 0
       velocity_pub.publish(vel_msg)
      
  else: # drive disabled
      vel_msg.linear.x = 0
      velocity_pub.publish(vel_msg)
    
    
  cv2.imshow("Image Window", bw_image)
  cv2.waitKey(3)
  
if __name__ == '__main__':
  rospy.init_node('follow_lane_gz', anonymous=True)           # follow_lane for simple_sim
  imgtopic = rospy.get_param("~imgtopic_name") # private name
  rospy.Subscriber(imgtopic, Image, image_callback)
  velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # '/cmd_vel' for simple_sim
  redL_sub = rospy.Subscriber("/redL_2", Bool, redL_cb, queue_size=1);
  srv = Server(FollowLaneConfig, dyn_rcfg_cb)
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
