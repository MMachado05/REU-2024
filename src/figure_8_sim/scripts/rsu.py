#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import cv2  # OpenCV library
import numpy as np

# stoplight simulation
# lights change every 10 seconds
# only one light for both directions so far

bridge = CvBridge()

def stop_light():

    #  publishers
    stop_pub = rospy.Publisher('/robot1/stoplight', Bool, queue_size=1)
    time_pub = rospy.Publisher('/robot1/time', Float64, queue_size=1)
    state_pub = rospy.Publisher('/states', Image, queue_size=1)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('figure_8_sim')
    state1_path = f"{package_path}/map/state1.png"
    state2_path = f"{package_path}/map/state2.png"  # Assuming you meant to have different images

    # Load images
    state1 = cv2.imread(state1_path)
    state2 = cv2.imread(state2_path)

    try:
        state1_msg = bridge.cv2_to_imgmsg(state1, encoding="bgr8")
        state2_msg = bridge.cv2_to_imgmsg(state2, encoding="bgr8")

    except CvBridgeError as e:
        rospy.logerr(f"Error converting images: {e}")
        return

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        curr = time.time()

        # Red at horizontal intersections, green at vertical intersections
        while (time.time() - curr) < 10:
            stop = Bool(data=True)
            left_time = Float64(data=10 - (time.time() - curr))  # Time remaining
            stop_pub.publish(stop)
            time_pub.publish(left_time)
            state_pub.publish(state2_msg)
            rate.sleep()

        curr = time.time()

        # Green at horizontal intersections, red at vertical intersections
        while (time.time() - curr) < 10:
            stop = Bool(data=False)
            left_time = Float64(data=10 - (time.time() - curr))  # Time remaining
            stop_pub.publish(stop)
            time_pub.publish(left_time)
            state_pub.publish(state1_msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('rsu')
    stop_light()

# to see stoplight visualization run rosboard and open up a browsing window with the IP http://127.0.0.1:8888/