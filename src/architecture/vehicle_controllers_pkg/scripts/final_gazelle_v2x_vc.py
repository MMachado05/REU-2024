#!/usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import Twist
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

def dyn_rcfg_cb(config, level):
    global drive_on, speed, twist_multiplier

    drive_on = config.drive_on
    speed = config.speed
    twist_multiplier = config.twist_multiplier

    return config

def follow_lane_cb(lane_twist_msg):
    global drive_on, speed, twist_multiplier, is_driving, velocity_msg
    if not drive_on:
        if is_driving:
            rospy.loginfo('final_gazelle_v2x_vc:81 - stopping vehicle')
            is_driving = False
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
        velocity_publisher.publish(velocity_msg)
        return
    
    if not is_driving:
        rospy.loginfo('final_gazelle_v2x_vc:89 - starting vehicle')
        is_driving = True
    velocity_msg.linear.x = speed
    velocity_msg.angular.z = -math.atan2(lane_twist_msg.angular.x, lane_twist_msg.angular.y) * twist_multiplier

    velocity_publisher.publish(velocity_msg)


if __name__ == "__main__":
    rospy.init_node("final_gazelle_v2x_vc")

    # Params
    lane_twist_in_topic = rospy.get_param("~lane_twist_in_topic")
    cmd_vel_out_topic = rospy.get_param("~cmd_vel_out_topic")

    # Subscribers
    lane_twist_subscriber = rospy.Subscriber(lane_twist_in_topic, Twist, follow_lane_cb)
    
    # Publishers
    velocity_publisher = rospy.Publisher(cmd_vel_out_topic, Twist, queue_size=1)

    # Dynamic reconfigure
    dyn_rcfg_srv = Server(GazelleNoYNoRNoGConfig, dyn_rcfg_cb)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
