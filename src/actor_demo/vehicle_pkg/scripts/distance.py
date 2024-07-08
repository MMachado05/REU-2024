#!/usr/bin/env python3

import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from follow_line_pkg.cfg import FollowLineConfig  
from geometry_msgs.msg import Twist
import cv2 as cv
import numpy as np
from std_msgs.msg import Bool, Float64, Time, Empty
from nav_msgs.msg import Odometry
import tkinter as tk
import math
import yaml
import rospkg

class Break(Exception): pass

# DYNAMIC SPEED ALGORITHM
# compute distance to intersection and adjust speed if needed
def compute_speed_to_intersection():
    global intersections, left_time, current_pose, waypoints, stop, stop_horiz, stop_vert, vel
    
    if not current_pose or not left_time or vel == 0:
        return 5.0 

    # find the closest waypoint to the current position
    closest_idx = -1
    min_distance = float('inf')
    for i, waypoint in enumerate(waypoints):
        dist = math.sqrt((current_pose['x'] - waypoint[0])**2 + (current_pose['y'] - waypoint[1])**2)
        if dist < min_distance:
            min_distance = dist
            closest_idx = i

    # sum the total distance to the next intersection
    total_distance = 0.0
    try:
        for i in range(closest_idx, len(waypoints) - 1):
            total_distance += math.sqrt((waypoints[i][0] - waypoints[i + 1][0])**2 + ((waypoints[i][1] - waypoints[i + 1][1])**2))
            for int in intersections:
                if waypoints[i + 1] == int[:2]:
                    direction = int[2]
                    #print(f"{int[3]} light")
                    raise Break
    except Break:
        pass
              
    if direction == 0:
        stop = stop_horiz
    else:
        stop = not stop_horiz
    
    t = total_distance / vel # time left to arrive at intersection with current speed

    if stop: # light is red
        if t < 10 and vel == 5.0: # if time to arrive less than changing to green light
            return float(total_distance / 10) - 0.5 # adjust speed
        else:
            return 5.0 # else keep speed

    else: # light is green
        # if t > 10 and t < 20 and vel == 5.0:
        #     return float(total_distance / 20) 
        return 5.0

# main method
if __name__ == '__main__':
    rospy.init_node('distance_node', anonymous=True)

    # -- Waypoints File Finder --
    lane = rospy.get_param("~lane_name") 
    yaml_file_path = f"/waypoints/{lane}.yaml"

    # load waypoints from the YAML file
    with open(yaml_file_path, 'r') as file:
        config = yaml.safe_load(file)

    # intersection and waypoint positions from YAML
    intersections = config['intersections']
    waypoints = config['waypoints']
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
