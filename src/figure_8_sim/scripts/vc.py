#!/usr/bin/env python3

import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from figure_8_sim.cfg import FollowLaneConfig  
from geometry_msgs.msg import Twist
import cv2 as cv
import numpy as np
from std_msgs.msg import Bool, Float64
from nav_msgs.msg import Odometry
import tkinter as tk
import math
import yaml
import rospkg

# veihicle control
# gets light status, intersection detection, and angular speed
# publishes angular and linear speeds as a Twist message

# variables
vel_msg = Twist()
bridge = CvBridge()
check = False
start_time = None
stop = None
left_time = None
current_pose = None
canvas = None
stoplight = None
timer_text = None
vel = 5.0
angular_vel = 0.0
yellow = False
check = False

# find the path to the YAML file within the package
rospack = rospkg.RosPack()
package_path = rospack.get_path('figure_8_sim')
yaml_file_path = f"{package_path}/map/waypoints.yaml"

# load waypoints from the YAML file
with open(yaml_file_path, 'r') as file:
    config = yaml.safe_load(file)

# intersection and waypoint positions from YAML
intersections = config['intersections']
waypoints = config['waypoints']

# dynamic reconfigure
def dyn_rcfg_cb(config, level):
    global speed, drive
    speed = config.speed
    drive = config.enable_drive
    return config

# CALLBACK FUNCTIONS
# stop light callback
def stop_cb(msg):
    global stop, vel
    temp = stop
    stop = msg.data
    if temp != stop:
        # adjust speed when light changes
        vel = compute_speed_to_intersection()
    motion()

# light timer callback
def time_cb(msg):
    global left_time
    left_time = 10 - msg.data # count down (periods of 10s)

# vehicle position callback
def pose_cb(pose):
    global current_pose
    current_pose = {'x': pose.pose.pose.position.x, 'y': pose.pose.pose.position.y}

# yellow/intersection callback
def yellow_cb(y):
    global yellow
    yellow = y.data

# angular speed callback
def angular_vel_cb(gap):
    global angular_vel
    angular_vel = gap.data

# DYNAMIC SPEED ALGORITHM
# compute distance to intersection and adjust speed if needed
def compute_speed_to_intersection():
    global intersections, left_time, current_pose, waypoints, stop, vel
    
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
    for i in range(closest_idx, len(waypoints) - 1):
        total_distance += math.sqrt((waypoints[i][0] - waypoints[i + 1][0])**2 + ((waypoints[i][1] - waypoints[i + 1][1])**2))
        if waypoints[i + 1] in intersections:
            break

    if stop: # light is red
        t = total_distance / vel # time left to arrive at intersection with current speed
        if t < 10: # if time to arrive less than changing to green light
            return float(total_distance / 10 - 1) # adjust speed (-1 to ensure arriving a bit early)
        else:
            return 5.0 # else keep speed
    else: # light is green
        t = total_distance / vel # time left to arrive at intersection with current speed
        if t > 10: # if time to arrive more than changing to red light
            return float(total_distance / 20 - 1) # adjust speed for next green (-1 to ensure arriving a bit early)
        else:
            return 5.0 # else keep speed

# make vehicle move
def motion():
    global vel, angular_vel, yellow, stop, check, start_time, current_time

    # linear speed
    current_time = time.time()
    # if arrive at intersection at red stop for safety
    if yellow and stop:
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
    # if arrive at intersection at green go straight
    elif yellow and not stop and not check:
        vel_msg.linear.x = 5.0
        vel_msg.angular.z = 0.0
        check = True
        start_time = time.time()
    # go straight for some time
    elif start_time:
        if abs(current_time - start_time) < 3.5:
            pass
        else:
            start_time = None
            check = False
    # other cases just go at vel
    else:
        if drive:
            vel_msg.linear.x = vel
        else:
            vel_msg.linear.x = 0

        # angular speed
        vel_msg.angular.z = angular_vel * vel
    
    # publish speed
    velocity_pub.publish(vel_msg)

# main method
if __name__ == '__main__':
    rospy.init_node('vc_node', anonymous=True)
    odom = rospy.get_param("~pose_name") 
    rospy.Subscriber(odom, Odometry, pose_cb)
    stop_sub = rospy.Subscriber("/robot1/stoplight", Bool, stop_cb, queue_size=1)
    time_sub = rospy.Subscriber("/robot1/time", Float64, time_cb, queue_size=1)
    yellow_sub = rospy.Subscriber("/robot1/yellow", Bool, yellow_cb, queue_size=1)
    angular_vel_sub = rospy.Subscriber("/robot1/angular_vel", Float64, angular_vel_cb, queue_size=1)
    velocity_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
    srv = Server(FollowLaneConfig, dyn_rcfg_cb)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass