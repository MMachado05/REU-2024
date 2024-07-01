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

# dbw commands
import math
from dataspeed_ulc_msgs.msg import UlcCmd # Drive by wire UL 
from dbw_polaris_msgs.msg import SteeringCmd # drive by wire native messages

# veihicle control
# gets light status, intersection detection, and angular speed
# publishes angular and linear speeds as a Twist message

# variables
vel_msg = Twist()
bridge = CvBridge()
check = False
start_time = None
start_time2 = None
stop = None
stop_horiz = None
stop_vert = None
left_time = None
current_pose = None
canvas = None
stoplight = None
timer_text = None
vel = 5.0
angular_vel = 0.0
yellow = False
check = False
lane = None
waypoints = None
intersections = None
direction = 0
angle = 0

class Break(Exception): pass

# dynamic reconfigure
def dyn_rcfg_cb(config, level):
    global speed, drive
    speed = config.speed
    drive = config.enable_drive
    return config

# CALLBACK FUNCTIONS
# stop light callback
# horizontal
# def stop_cb0(msg):
#     global stop_horiz, vel
#     temp = stop_horiz
#     stop_horiz = msg.data


#     if temp != stop_horiz:
#         # adjust speed when light changes
#         vel = compute_speed_to_intersection()
#     motion()

# state callback
def state_cb(msg):
    global stop
    stop = not msg.data

    if stop:
        print('red')
    else:
        print('stop')
    
    # motion()

# light timer callback
def time_to_state_cb(msg):
    global left_time
    left_time = str(msg.data.secs) + '.' + str(msg.data.nsecs) # count down (periods of 10s)
    print(left_time)

    motion()

# yellow/intersection callback
def yellow_cb(y):
    global yellow
    yellow = y.data

# angular speed callback
def angle_cb(ang):
    global angle
    angle = ang.data

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

# make vehicle move
def motion():
    global vel, angular_vel, yellow, stop, check, start_time, current_time, start_time2

    if drive:
        if start_time2 is not None:
            current_time = time.time()
            if (current_time - start_time2) < 20:
                publish_ulc_speed(speed)
            else:
                start_time2 = None
        elif yellow and stop and start_time == None:
            start_time = time.time()
        elif start_time is not None:
            # current_time = time.time()
            if stop:
                publish_ulc_speed(0)
            else:
                start_time = None
                start_time2 = time.time()
        else:
            publish_ulc_speed(speed)
    else:
        publish_ulc_speed(0)
    
    # publish speed
    # velocity_pub.publish(vel_msg)
    publish_steering(angle)
    enable_dbw()

def publish_ulc_speed(speed: float) -> None:
    """Publish requested speed to the vehicle using ULC message."""

    ulc_cmd = UlcCmd()
    ulc_cmd.enable_pedals = True
    ulc_cmd.enable_steering = False  # NOTE: Steering control via ULC is not used here
    ulc_cmd.enable_shifting = True
    ulc_cmd.shift_from_park = True

    ulc_cmd.linear_velocity = round((speed / 2.237), 3)  # Convert mph to m/s
    ulc_cmd.linear_accel = 0.0
    ulc_cmd.linear_decel = 0.0
    ulc_cmd.jerk_limit_throttle = 0.0
    ulc_cmd.jerk_limit_brake = 0.0

    ulc_cmd.pedals_mode = 0  # Speed mode
    # ---------------------------------------------------------------------
    pub_ulc.publish(ulc_cmd)


def publish_steering(requested_road_angle: float = None) -> None:
    """Publish requested steering to the vehicle.
    Input can be desired degree road angle (-37 to 37) or steering angle (-600 to 600)"""

    if requested_road_angle is None:
        rospy.logerr("publish_steering called with no steering angle provided")
        requested_road_angle = 0

    if requested_road_angle is not None:
        if requested_road_angle < 0:
            requested_road_angle = max(requested_road_angle, -37) * 16.2
        else:
            requested_road_angle = min(requested_road_angle, 37) * 16.2

    # Make steering message -----------------------------------------------
    msg_steering = SteeringCmd()
    msg_steering.steering_wheel_angle_cmd = math.radians(requested_road_angle)
    # NOTE: (-600deg to 600deg converted to radians)
    msg_steering.enable = True  # Enable Steering, required 'True' for control via ROS

    # Do NOT use these without completely understanding how they work on the hardware level:
    msg_steering.cmd_type = SteeringCmd.CMD_ANGLE  # CAUTION: Torque mode disables lateral acceleration limits
    # Use angle velocity to control rate. Lock to lock = 1200deg i.e. 300deg/s will be 4secs lock to lock
    msg_steering.steering_wheel_angle_velocity = math.radians(300)  # deg/s -> rad/s
    msg_steering.steering_wheel_torque_cmd = 0.0  # Nm
    msg_steering.clear = False
    msg_steering.ignore = False
    msg_steering.calibrate = False
    msg_steering.quiet = False
    msg_steering.count = 0
    # ---------------------------------------------------------------------
    pub_steering.publish(msg_steering)


def enable_dbw() -> None:
    """Enable vehicle control using ROS messages"""
    msg = Empty()
    pub_enable_cmd.publish(msg)

# main method
if __name__ == '__main__':
    rospy.init_node('vc_node', anonymous=True)

    print('running...')

    
    # odom = rospy.get_param("~pose_name") 
    # rospy.Subscriber(odom, Odometry, pose_cb)
    # stop_sub0 = rospy.Subscriber("/stoplight1", Bool, stop_cb0, queue_size=1)
    # stop_sub1 = rospy.Subscriber("/stoplight2", Bool, stop_cb1, queue_size=1)
    # time_sub = rospy.Subscriber("/time", Float64, time_cb, queue_size=1)
    # yellow_sub = rospy.Subscriber("yellow", Bool, yellow_cb, queue_size=1)
    # angular_vel_sub = rospy.Subscriber("angular_vel", Float64, angular_vel_cb, queue_size=1)
    # velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    light_ns = rospy.get_param("~light_ns")
    time_to_state_sub = rospy.Subscriber('/' + light_ns + '/time_to_state', Time, time_to_state_cb, queue_size=1)
    state_sub = rospy.Subscriber('/' + light_ns + '/state', Bool, state_cb,queue_size=1)
    yellow_sub = rospy.Subscriber('yellow', Bool, yellow_cb, queue_size=1)
    angle_sub = rospy.Subscriber('angle', Float64, angle_cb, queue_size=1)
    velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # dbw pusblishers
    pub_ulc = rospy.Publisher("/vehicle/ulc_cmd", UlcCmd, queue_size=1)
    pub_steering = rospy.Publisher("/vehicle/steering_cmd", SteeringCmd, queue_size=1)
    pub_enable_cmd = rospy.Publisher("/vehicle/enable", Empty, queue_size=1)



    srv = Server(FollowLineConfig, dyn_rcfg_cb)

    # -- Waypoints File Finder --

    # rospack = rospkg.RosPack()
    # package_path = rospack.get_path('figure_8_sim')
    # lane = rospy.get_param("~lane_name") 
    # yaml_file_path = f"{package_path}/map/waypoints{lane}.yaml"

    # # load waypoints from the YAML file
    # with open(yaml_file_path, 'r') as file:
    #     config = yaml.safe_load(file)

    # # intersection and waypoint positions from YAML
    # intersections = config['intersections']
    # waypoints = config['waypoints']
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass