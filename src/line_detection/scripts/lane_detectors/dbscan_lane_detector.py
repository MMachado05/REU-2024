#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from line_detection.cfg import DBScanLaneDetectorConfig  # packageName.cfg
from std_msgs.msg import Float32
import cv2 as cv
import numpy as np
from sklearn.cluster import DBSCAN


class DBScanLaneDetector:
    """
    Density-based scan lane detector node.

    Logic taken from Beñat's "advanced_dbw_follow_lane.py" code.
    """

    preprocessed_img_subscriber: rospy.Subscriber
    center_offset_publisher: rospy.Publisher
    dyn_rcfg_srv: Server

    # ------------------------------------------------
    # ------- Internal state-related functions -------
    # ------------------------------------------------
    def __init__(self):
        """
        Initializes the node.
        """
        # Node architecture
        rospy.init_node("dbscan_lane_detector", anonymous=True)

        self.preprocessed_img_subscriber = rospy.Subscriber(
            rospy.get_param("~img_in_topic"), Image, self._preprocess_image
        )
        self.center_offset_publisher = rospy.Publisher(
            rospy.get_param("~center_offset_out_topic"), Float32, queue_size=1
        )
        self.dyn_rcfg_srv = Server(
            DBScanLaneDetectorConfig, self._dynamic_reconfig_callback
        )

    def _dynamic_reconfig_callback(self, config, _):
        """
        Callback function for dynamic reconfigure.
        """
        pass

    # ---------------------------------------
    # ----------- Lane detection ------------
    # ---------------------------------------
    def _find_lane(self, image: Image) -> None:
        """
        Detects a lane in the image and publishes the offset between the desired
        midpoit, and the current midpoint.
        """
        pass

if __name__ == "__main__":
    try:
        DBScanLaneDetector()
    except rospy.ROSInterruptException:
        pass
