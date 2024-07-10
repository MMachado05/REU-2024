#!/usr/bin/env python3

import rospkg
import yaml
import rospy
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Time, Bool
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from vehicle_controllers_pkg.cfg import GazelleNoYNoRNoGConfig

lane_twist_subscriber: rospy.Subscriber
velocity_publisher: rospy.Publisher
dyn_rcfg_srv: Server

drive_on = False
speed = 0.0
twist_multiplier = 1.0

is_driving = False

velocity_msg = Twist()

di = None
curent_pose = None
time_left = None
current_light_state = False
green_duration = 0
red_duration = 0

class Break (Exception):
    pass

def dyn_rcfg_cb(config, level):
    global drive_on, speed, twist_multiplier

    drive_on = config.drive_on
    speed = config.speed
    twist_multiplier = config.twist_multiplier

    return config

def green_duration_cb(msg):
    global green_duration
    green_duration = msg.data

def red_duration_cb(msg):
    global red_duration
    red_duration = msg.data

def time_to_state_cb(msg):
    global time_left
    time_left = float(str(msg.data.secs) + '.' + str(msg.data.nsecs))

def light_state_cb(msg):
    global current_light_state
    current_light_state = msg.data

def pose_cb(msg):
    global current_pose
    current_pose = {'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y}


def distance_to_intersection():
    global intersections, waypoints, total_distance

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
                    raise Break
    except Break:
        pass

    rospy.loginfo('total_distance: ' + str(total_distance))

    return total_distance

def calculate_speed_to_intersection():
    global di

    # Benat if not and or statement
    # if not current_pose or not time_left or di == None:
    #     return speed
    
    di = distance_to_intersection()

    if current_light_state == True: # light is green
        rospy.loginfo('green')
        return speed
    else: # light is red
        rospy.loginfo('red')

        new_speed = di / time_left
        
        if new_speed > speed:
            return speed
        else:
            return new_speed
        
def follow_lane_cb(lane_twist_msg):
    global drive_on, speed, twist_multiplier, is_driving, velocity_msg

    if not drive_on:
        if is_driving:
            rospy.loginfo('final_gazelle_v2x_vc:121 - stopping vehicle')
            is_driving = False
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
        velocity_publisher.publish(velocity_msg)
        return
    
    if not is_driving:
        rospy.loginfo('final_gazelle_v2x_vc:129 - starting vehicle')
        is_driving = True

    velocity_msg.linear.x = calculate_speed_to_intersection()
    velocity_msg.angular.z = -math.atan2(lane_twist_msg.angular.x, lane_twist_msg.angular.y) * twist_multiplier

    velocity_publisher.publish(velocity_msg)

if __name__ == "__main__":
    rospy.init_node("final_gazelle_v2x_vc")

    # Params
    lane_twist_in_topic = rospy.get_param("~lane_twist_in_topic")
    cmd_vel_out_topic = rospy.get_param("~cmd_vel_out_topic")
    lane_name = rospy.get_param("~lane_name") 

    # Subscribers
    lane_twist_subscriber = rospy.Subscriber(lane_twist_in_topic, Twist, follow_lane_cb)
    green_duration_sub = rospy.Subscriber('/light/green_duration', Int32, green_duration_cb)
    red_duration_sub = rospy.Subscriber('/light/red_duration', Int32, red_duration_cb)
    time_to_next_state_sub = rospy.Subscriber(f'/light/{lane_name}/time_to_next_state', Time, time_to_state_cb)
    light_state_sub = rospy.Subscriber(f'/light/{lane_name}/state', Bool, light_state_cb)
    odom_sub = rospy.Subscriber("odom", Odometry, pose_cb)

    # Publishers
    velocity_publisher = rospy.Publisher(cmd_vel_out_topic, Twist, queue_size=1)

    # Dynamic reconfigure
    dyn_rcfg_srv = Server(GazelleNoYNoRNoGConfig, dyn_rcfg_cb)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('v2x_simulator')
    yaml_file_path = f"{package_path}/waypoints/{lane_name}.yaml"

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
