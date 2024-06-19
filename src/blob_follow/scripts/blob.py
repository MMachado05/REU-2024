#!/usr/bin/env python3

from distutils.log import debug
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from typing import Dict, Tuple, List
import cv2 as cv
from dynamic_reconfigure.server import Server
from blob_follow.cfg import BlobConfig
import math
import numpy as np
from numpy import ndarray
from lane_helpers.vec import Vec
from lane_helpers.lane_centering import center_lane
from lane_helpers.lane_detection import find_lanes, compute_lines
from lane_helpers.utils import rows, cols


class LaneFollowBlob:

    def __init__(self):
        rospy.logwarn('LaneFollowBlob (py)')
        self.cvbridge = CvBridge()

        self.twist_pub = rospy.Publisher(rospy.get_param('~twist_topic'), Twist, queue_size=10)

        self.debug_publishers: Dict[str, rospy.Publisher] = {}
        self.config = None # Dynamic reconfigure
        self.dynamic_reconfigure_server = Server(BlobConfig, self.dynamic_reconfigure_callback)

        # Let's go!
        rospy.Subscriber(rospy.get_param('~image_topic', '/image'), Image, self.input_callback, queue_size=1)


    def dynamic_reconfigure_callback(self, config, level):
        rospy.logwarn('Got config!')
        self.config = config
        return config


    def input_callback(self, msg: Image):
        #rospy.loginfo('Got image')
        if not self.config:
            rospy.logwarn('Waiting for config..')
            return

        image = self.cvbridge.imgmsg_to_cv2(msg)
        twist = self.process(image)
        self.twist_pub.publish(twist)


    def debug_publish(self, name, image: ndarray):
        name = f'lane_follow_blob_debug/{name}'
        if name not in self.debug_publishers:
            self.debug_publishers[name] = rospy.Publisher(name, Image, queue_size=2)
        self.debug_publishers[name].publish(self.cvbridge.cv2_to_imgmsg(image))


    def make_twist(self, turn: float) -> Twist:
        """ convert adjustment value to twist message"""
        config = self.config
        twist = Twist()
        twist.linear.x  = config.drive_speed
        twist.angular.z = -config.blob_mult * turn

        if not config.enable_drive:
            twist.linear.x  = 0
            twist.angular.z = 0
        elif not config.enable_forward:
            twist.linear.x = 0

        return twist


    def process(self, input_image: ndarray) -> Twist:
        # Create a copy of the input to draw debug data on
        debug_image = input_image.copy()

        # Find the lanes in the image
        lanes_image = find_lanes(input_image, self.config, debug_image=debug_image)
        self.debug_publish('lanes_image', lanes_image)

        # Run blob lane centering algorithm
        p0 = Vec(cols(lanes_image)/2, rows(lanes_image) - rows(lanes_image)/10)
        p_diff = center_lane(lanes_image, p0, debug_image=debug_image)
        adjust = p_diff.x
        #rospy.loginfo(f'force vector: {p_diff}')

        self.debug_publish('debug_final', debug_image)
        # Convert the output to a Twist message
        return self.make_twist(adjust)


rospy.init_node('blob')
blob = LaneFollowBlob()
rospy.spin()