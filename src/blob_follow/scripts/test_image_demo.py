#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import cv2 as cv
from dynamic_reconfigure.server import Server
from lane_follow_blob.cfg import BlobConfig
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from numpy import ndarray
from lane_helpers.vec import Vec
from lane_helpers.lane_centering import center_lane
from lane_helpers.lane_detection import find_lanes
from lane_helpers.utils import rows, cols



class TestImageDemo:

    def __init__(self):
        self.bridge = CvBridge()
        self.enable_drive = True
        self.speed = 1
        
        self.cam_sub = rospy.Subscriber("camera/image_raw", Image, self.load_image)
        self.velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.config = None # Dynamic reconfigure
        self.dynamic_reconfigure_server = Server(BlobConfig, self.dynamic_reconfigure_callback)
        

    def load_image(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            self.process(cv_image)
        except CvBridgeError as e:
            print(e)
        

    def dynamic_reconfigure_callback(self, config, level):
        rospy.logwarn('Got config!')
        self.enable_drive = config.enable_drive
        self.speed = config.drive_speed
        self.config = config
        # self.process(self.image)
        return config


    # def change_image_cb(self, val):
    #     print(val)
    #     image = self.load_image(val)
    #     if image is not None:
    #         self.image = image
    #         self.process(self.image)


    def process(self, input_image: ndarray) -> Twist:
        # Create a copy of the input to draw debug data on
        debug_image = input_image.copy()
        cv.imshow('Original', debug_image)

        # Find the lanes in the ilanes_imagemage
        lanes_image = find_lanes(input_image, self.config, debug_image=debug_image)
        self.last_image_1 = lanes_image.copy()

        # Run blob lane centering algorithm
        if rospy.get_param('~draw_blob', False):
            p0 = Vec(cols(lanes_image)/2, rows(lanes_image) - rows(lanes_image)/10)
            p_diff = center_lane(lanes_image, p0, debug_image=debug_image, iters=200)
            adjust = p_diff.x
            #rospy.loginfo(f'force vector: {p_diff}')

        self.last_debug_image = debug_image
        cv.imshow('Lanes Image', self.last_image_1)
        cv.waitKey(3)
        self.publish_vel(adjust, 0.00002)
    
    # publishes velocity to a car
    def publish_vel(self, adjust, tol):
        ang_speed = 1
        vel_msg = Twist()
        if self.enable_drive:
            vel_msg.linear.x = self.speed
            if (adjust + tol < 0):
                vel_msg.angular.z = -1 * adjust * 5
            elif (adjust - tol > 0):
                vel_msg.angular.z = -1 * adjust * 5
            else:
                vel_msg.angular.z = 0
        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
        self.velocity_pub.publish(vel_msg)


rospy.init_node('test_image_demo')
demo = TestImageDemo()
rospy.spin()


# while not rospy.is_shutdown():
#     if demo.last_debug_image is not None:
#         cv.imshow('final', demo.last_debug_image)
#         if not window_setup_done:
#             print('assing trackbar')
#             cv.createTrackbar('image_id', 'final', 1, 8, demo.change_image_cb)
#             window_setup_done = True

#     if demo.last_image_1 is not None:
#         cv.imshow('med', demo.last_image_1)

#     cv.waitKey(1)
