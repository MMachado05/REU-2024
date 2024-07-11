#!/usr/bin/env python3

import rospkg
import yaml
import rospy
import math
import cv2

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Time, Bool
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from vehicle_controllers_pkg.cfg import GazelleNoYNoRNoGConfig
from cv_bridge import CvBridge, CvBridgeError

lane_twist_subscriber: rospy.Subscriber
velocity_publisher: rospy.Publisher
dyn_rcfg_srv: Server

drive_on = False
speed = 0.0
twist_multiplier = 1.0

is_driving = False

velocity_msg = Twist()
bridge = CvBridge()

di = None
curent_pose = None
time_left = None
current_light_state = False
state = 'RED'
green_duration = 15.0
red_duration = 15.0
curr_vel = None
vel = 0.0
d_int = 0.0
d_int_end = 0.0
d_red = 0.0
d_green = 0.0
d_green_red = 0.0
v = 0.0

class Break (Exception):
    pass

def dyn_rcfg_cb(config, level):
    global drive_on, speed, twist_multiplier, curr_vel

    drive_on = config.drive_on
    speed = config.speed
    twist_multiplier = config.twist_multiplier

    rospy.loginfo("i've been changed!!!")

    return config

def green_duration_cb(msg):
    global green_duration
    green_duration = msg.data

def red_duration_cb(msg):
    global red_duration
    red_duration = msg.data

def time_to_state_cb(msg):
    global time_left, vel
    time_left = float(str(msg.data.secs) + '.' + str(msg.data.nsecs))

    # EXCEPTION HANDLING
    # ERROR IN RSU NODES, CAUSES THIS TOPIC TO NO RUNNING THE CB
    
    # rospy.loginfo("i'm getting called!!!")

    # try:
    #     pass
    # except Exception as e:
    #     rospy.logerr('Exception in callback')

def light_state_cb(msg):
    global current_light_state, state, vel
    current_light_state = msg.data

    if current_light_state:
        state = 'GREEN'
        light_state_img_pub.publish(green_img_msg)
    else:
        state = 'RED'
        light_state_img_pub.publish(red_img_msg)

    vel = calculate_speed_to_intersection()

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

    # rospy.loginfo('total_distance: ' + str(total_distance))

    return total_distance

def calculate_speed_to_intersection():
    global d_int, v, curr_vel, v, dist_int_end # d_red, d_green, d_green_red

    dist_tolerance = 0.0 # whatever units this sim uses
    time_tolerance = 1.5 # seconds -- future be adjusted based on speed of the car to account the amount of time it takes the real car to slow down

    intersection_distance = 8.0

    v = 0.0
    d_int = distance_to_intersection() - dist_tolerance
    d_int_end = d_int + intersection_distance


    if current_light_state is False: # light is RED

        d = speed * red_duration

        if d > d_int:  # if we are going too fast
            v = d_int / (red_duration + time_tolerance)
        else:
            v = speed
    else:  # light is GREEN

        d = speed * green_duration

        if d > d_int_end: # if you can make the intersection before it turns red
            v = speed
        else:

            d = speed * (green_duration + red_duration + time_tolerance)

            if d < d_int: # if the you will make the intersection once the light turns green, AGAIN
                v = speed
            else:
                v = d_int / (green_duration + red_duration + time_tolerance)

    
    rospy.loginfo(f'\n state: {state} \n dist to intersection: {d_int} \n dist to int end: {d_int_end} \n dist calc: {d} \n orginal speed: {speed} \n calculated speed: {v} \n\n')
    return v
        
def follow_lane_cb(lane_twist_msg):
    global drive_on, speed, twist_multiplier, is_driving, velocity_msg, vel

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
        distance_to_intersection()

    velocity_msg.linear.x = vel
    velocity_msg.angular.z = -math.atan2(lane_twist_msg.angular.x, lane_twist_msg.angular.y) * twist_multiplier

    velocity_publisher.publish(velocity_msg)

def open_waypoints():
    global intersections, waypoints

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('v2x_simulator')
    yaml_file_path = f"{package_path}/waypoints/{lane_name}.yaml"

    # load waypoints from the YAML file
    with open(yaml_file_path, 'r') as file:
        config = yaml.safe_load(file)

    # intersection and waypoint positions from YAML
    intersections = config['intersections']
    waypoints = config['waypoints']

    return

def open_traffic_light_images():
    global green_img_msg, red_img_msg

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('v2x_simulator')

    green_img_path = f"{package_path}/pics/green_traffic_light.png"
    red_img_path = f"{package_path}/pics/red_traffic_light.png"

    green_img = cv2.imread(green_img_path)
    red_img = cv2.imread(red_img_path)


    try:
        green_img_msg = bridge.cv2_to_imgmsg(green_img, "bgr8")
        red_img_msg = bridge.cv2_to_imgmsg(red_img, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f'Error: {e}')
        return
    
    return


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
    light_state_img_pub = rospy.Publisher(f'/light/{lane_name}/image', Image, queue_size=1)

    # Dynamic reconfigure
    dyn_rcfg_srv = Server(GazelleNoYNoRNoGConfig, dyn_rcfg_cb)

    open_waypoints()
    open_traffic_light_images()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass