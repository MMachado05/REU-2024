#!/usr/bin/env python3
# Stop at Crosswalk

import rospy
from sensor_msgs.msg import Image, NavSatFix
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from follow_lane_pkg.cfg import FollowLaneConfig
from geometry_msgs.msg import Twist
import cv2 as cv
import numpy as np
from flask import Flask, render_template, jsonify
from flask_cors import CORS
import threading

# Flask app setup
app = Flask(__name__)
CORS(app)

# Global variable to store ROS node information
ros_info = {"linear": 0, "angular": 0, "gps": {"latitude": 0, "longitude": 0}, "pose": {"x": 0, "y": 0}}

vel_msg = Twist()
bridge = CvBridge()
check = False
speed = 0
enable = False

def dyn_rcfg_cb(config, level):
    global speed, enable
    speed = config.speed
    enable = config.enable_drive
    return config

def image_callback(ros_image):
    global check, bridge, ros_info, vel_msg, enable
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    rows, cols, channels = cv_image.shape
    cv_image = cv_image[rows//2:, :]
    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    thresh = cv.inRange(hsv, lower_white, upper_white)
    contours, _ = cv.findContours(thresh, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    cv.drawContours(cv_image, contours, -1, (0, 0, 255), 10)

    max_area = 0
    max_c = None
    for c in contours:
        area = cv.contourArea(c)
        if area > max_area:
            max_area = area
            max_c = c

    if max_c is not None:
        M = cv.moments(max_c)
        if M['m00'] != 0:
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            cx -= 200
            cv.circle(cv_image, (cx, cy), 10, (0, 0, 0), -1)
            cv.imshow('Contours with centroid dot', cv_image)
            cv.waitKey(3)

            mid = cols / 2
            linear_threshold1 = 50
            linear_threshold2 = 80
            if enable:
                if abs(cx - mid) < linear_threshold1:
                    vel_msg.linear.x = 5.0
                    ros_info["linear"] = 5.0
                elif abs(cx - mid) < linear_threshold2:
                    vel_msg.linear.x = 3.5
                    ros_info["linear"] = 3.5
                else:
                    vel_msg.linear.x = 2.0
                    ros_info["linear"] = 2.0

                angular_threshold = 20
                if mid < cx - angular_threshold:
                    vel_msg.angular.z = 1.2 * -abs((mid - cx) / mid)
                    ros_info["angular"] = 1.2 * -abs((mid - cx) / mid)
                elif mid > cx + angular_threshold:
                    vel_msg.angular.z = 1.2 * abs((mid - cx) / mid)
                    ros_info["angular"] = 1.2 * abs((mid - cx) / mid)
            else:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                ros_info["linear"] = 0
                ros_info["angular"] = 0
            velocity_pub.publish(vel_msg)

def gps_callback(gps):
    global ros_info
    ros_info["gps"] = {"latitude": gps.latitude, "longitude": gps.longitude}

def odom_callback(odom):
    global ros_info
    ros_info["pose"] = {"x": odom.pose.pose.position.x, "y": odom.pose.pose.position.y}

@app.route('/')
def index():
    global ros_info
    return render_template('index.html', ros_info=ros_info)

@app.route('/linear_speed')
def get_linear():
    global ros_info
    return jsonify({"linear": ros_info["linear"]})

@app.route('/angular_speed')
def get_angular():
    global ros_info
    return jsonify({"angular": ros_info["angular"]})

@app.route('/location')
def get_location():
    global ros_info
    return jsonify({"gps": {
            "latitude": ros_info["gps"]["latitude"],
            "longitude": ros_info["gps"]["longitude"]
        }})

@app.route('/position')
def get_position():
    global ros_info
    return jsonify({"pose": {
            "x": ros_info["pose"]["x"],
            "y": ros_info["pose"]["y"]
        }})

def flask_thread():
    app.run()

if __name__ == '__main__':
    rospy.init_node('node1', anonymous=True)
    imgtopic = rospy.get_param("~imgtopic_name")
    gps = rospy.get_param("~gps_name")
    odom = rospy.get_param("~pos_name")
    rospy.Subscriber(imgtopic, Image, image_callback)
    rospy.Subscriber(gps, NavSatFix, gps_callback)
    rospy.Subscriber(odom, Odometry, odom_callback)
    velocity_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
    srv = Server(FollowLaneConfig, dyn_rcfg_cb)

    # Start the Flask app in a separate thread
    threading.Thread(target=flask_thread).start()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
