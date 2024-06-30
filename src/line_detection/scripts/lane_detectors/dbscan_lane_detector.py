#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from line_detection.cfg import DBScanLaneDetectorConfig  # packageName.cfg
from geometry_msgs.msg import Twist
import cv2 as cv
import numpy as np
from sklearn.cluster import DBSCAN

class DBScanLaneDetector:
    """
    Density-based scan lane detector node.

    Logic taken from Beñat's "advanced_dbw_follow_lane.py" code.
    """


if __name__ == "__main__":
    try:
        DBScanLaneDetector()
    except rospy.ROSInterruptException:
        pass
