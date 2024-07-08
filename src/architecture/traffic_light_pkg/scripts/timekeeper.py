#!/usr/bin/env python3

import rospy
from std_msgs.msg import Time

class Timekeeper:
    """
    A class to make up for the fact that I have no idea how to rectify wanting
    a `while not rospy.is_shutdown()` loop *and* a `rospy.spin()` call in my traffic
    light code.
    """
