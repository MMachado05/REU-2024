#!/usr/bin/env python3

import rospy
from geopy.distance import geodesic
from libsbp_ros_msgs.msg import MsgPosLlh
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

global prev_coords, velocity, acceleration, dx_list, dv_list, a_list

prev_coords = None
velocity = 0.0
accerleration = 0.0

gps_latitude = 0.0
gps_longitude = 0.0

dx_list = []
dv_list = []
a_list = []

def imu_cb(msg):
    global imu_angular_z

    imu_angular_z = msg.angular_velocity.z
    imu_angular_z_pub.publish(imu_angular_z)

def gps_position_cb(msg):
    global gps_latitude, gps_longitude

    gps_latitude = msg.lat # y
    gps_longitude = msg.lon # x

def timer_callback(event):
    global prev_coords
    current_cords = (gps_latitude, gps_longitude)

    if prev_coords is not None:
        # Calculates distance between two coords
        distance = geodesic(prev_coords, current_cords).meters
        
        # Calculate velocity
        velocity = distance / duration if duration != 0 else 0

        # Append to lists
        dx_list.append(distance)
        dv_list.append(velocity)

        if len(dv_list) > 1:
            # Calculate acceleration
            acceleration = (dv_list[-1] - dv_list[-2]) / duration
            a_list.append(acceleration)

    prev_coords = current_cords

    if dx_list and dv_list and a_list:
        # Publish the most recent values from the lists
        velocity_pub.publish(dv_list[-1])
        acceleration_pub.publish(a_list[-1])
        #rospy.loginfo(f"Distance: {dx_list[-1]}, Velocity: {dv_list[-1]}, Acceleration: {a_list[-1]}")
    else:
        rospy.loginfo("Waiting for more data ...")



if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('comfortability_metrics', anonymous=True)
    print('running comfortability_metrics node ... ')

    global duration
    duration = 0.1 # in seconds

    # Publishers
    # distance_pub = rospy.Publisher('dist', Float64, queue_size=1)
    velocity_pub = rospy.Publisher('velocity', Float64, queue_size=1)
    acceleration_pub = rospy.Publisher('acceleration', Float64, queue_size=1)
    imu_angular_z_pub = rospy.Publisher('imu', Float64, queue_size=1)

    # Subscriber
    gps_sub = rospy.Subscriber("reference/piksi/position_receiver_0/sbp/pos_llh", MsgPosLlh, gps_position_cb, queue_size=1)
    imu_sub = rospy.Subscriber("vehicle/imu/data_raw", Imu, imu_cb, queue_size=1)

    # Call this function every t seconds (intervals)
    rospy.Timer(rospy.Duration(duration), timer_callback)

    # Keep the node running
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
