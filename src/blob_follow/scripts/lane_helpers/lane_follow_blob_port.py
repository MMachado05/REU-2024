#
# The follwing code is a more-or-less direct port of the original blob implementation
# https://github.com/LTU-Actor/Route-Blob
#

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

# A few helper functions and classes for making the port cleaner
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

Vec4i = Tuple[int, int, int, int]

def rows(mat: ndarray) -> int:
    return mat.shape[0]

def cols(mat: ndarray) -> int:
    return mat.shape[1]



class LaneFollowBlob:

    def __init__(self):
        self.cvbridge = CvBridge()

        self.twist_pub = rospy.Publisher('cmd', Twist, queue_size=10)

        self.debug_publishers: Dict[str, rospy.Publisher] = {}
        self.config = None # Dynamic reconfigure
        self.dynamic_reconfigure_server = Server(BlobConfig, self.dynamic_reconfigure_callback)

        # Let's go!
        rospy.Subscriber('/car/camera1/image_raw', Image, self.input_callback, queue_size=1)


    def spin(self):
        rospy.logwarn('Lane follow blob running!!')
        rospy.spin()


    def dynamic_reconfigure_callback(self, config, level):
        rospy.logwarn('Got config!')
        self.config = config
        return config


    def input_callback(self, msg: Image):
        rospy.loginfo('Got image')
        if not self.config:
            rospy.logwarn('Waiting for config..')
            return

        image = self.cvbridge.imgmsg_to_cv2(msg)
        twist = self.process(image)
        self.twist_pub.publish(twist)


    def debug_publish(self, name, image: ndarray):
        name = f'/lane_follow_blob_debug/{name}'
        if name not in self.debug_publishers:
            self.debug_publishers[name] = rospy.Publisher(name, Image, queue_size=2)
        self.debug_publishers[name].publish(self.cvbridge.cv2_to_imgmsg(image))


    def process(self, input_image: ndarray) -> Twist:
        # Create a copy of the input to draw debug data on
        debug_image = input_image.copy()
        # Find the lanes in the image
        lanes_image = self.find_lanes(input_image, debug_image=debug_image)
        self.debug_publish('lanes_image', lanes_image)
        # Run the blob algorithm
        adjust = self.blob_adjust(lanes_image, debug_image=debug_image)
        self.debug_publish('debug_final', debug_image)
        # Convert the output to a Twist message
        return self.make_twist(adjust)


    def make_twist(self, turn: float) -> Twist:
        config = self.config
        twist = Twist()
        twist.linear.x  = config.drive_speed;
        twist.angular.z = -config.blob_mult * turn;

        if not config.enable_drive:
            twist.linear.x  = 0;
            twist.angular.z = 0;
        elif not config.enable_forward:
            twist.linear.x = 0;

        return twist


    def find_lanes(self, input_image: ndarray, debug_image:ndarray=None):
        """
        This algorithm uses light-on-dark contrast to find 
        lane lines. If lanes do not have this property, another
        lane-finding algorithm may be used instead
        """
        self.debug_publish('echo', input_image)
        config = self.config

        # Get the value channel
        # This assumes bright-on-dark lanes
        hsv = cv.cvtColor(input_image, cv.COLOR_BGR2HSV)
        (h, s, v) = cv.split(hsv)

        # Median blur
        v = cv.medianBlur(v, config.enhance_blur * 2 + 1)
        image = cv.merge((h, s, v))
        image = cv.cvtColor(image, cv.COLOR_HSV2BGR)

        self.debug_publish('pre_canny', image)

        ## Find edges using Laplacian and Sobel
        # Canny could also be used here but the Laplacian/Sobel
        # approach typically yeilds improved expiremental
        # results for this case
        # image = cv.Laplacian(image, -1, config.lapla_ksize * 2 + 1)
        # image = cv.Sobel(image, -1, config.sobel_xorder,
        #             config.sobel_yorder,
        #             config.sobel_ksize * 2 + 1)
        image = cv.Canny(image, config.canny_lower_thresh,
              config.canny_upper_thresh,
              apertureSize=config.canny_aperture_size * 2 + 1)

        self.debug_publish('post_canny', image)

        if config.lines_enable:
            image = self.compute_lines(image)

        return image


    def compute_lines(self, image: ndarray) -> ndarray:
        config = self.config
        lines_mat = np.zeros_like(image)
        x = 0
        y = int(config.lines_top * rows(image))
        w = cols(image)
        h = int(rows(image) - config.lines_top * rows(image))
        image_cropped = image[y:y+h, x:x+w]
            

        self.debug_publish('image_cropped', image_cropped)
        lines = cv.HoughLinesP(image_cropped,
                               rho=config.lines_rho,
                               theta=0.01745329251,
                               threshold=config.lines_thresh,
                               minLineLength=config.lines_min_len,
                               maxLineGap=config.lines_max_gap)
        if lines is not None:
            for l in lines:
                l = l[0] # (4,1) => (4,)
                diffx = l[0] - l[2]
                diffy = l[1] - l[3]

                slope = diffy / diffx

                if abs(slope) < config.lines_min_slope: continue

                diffx *= 5
                diffy *= 5

                l[0] -= diffx
                l[1] -= diffy
                l[2] += diffx
                l[3] += diffy

                cv.line(
                    lines_mat,
                    (l[0], int(l[1] + config.lines_top * rows(image))),
                    (l[2], int(l[3] + config.lines_top * rows(image))),
                    255, 5)

        return lines_mat



    def blob_adjust(self, image: ndarray, debug_image:ndarray=None) -> float:
        """
        Essentially a direct port from the original c++ algorithm
        """
        config = self.config

        # Dilate images
        dilation_size = (2 * config.blob_dilation_size + 1, 2 * config.blob_dilation_size + 1)
        dilation_anchor = (config.blob_dilation_size, config.blob_dilation_size)
        dilate_element = cv.getStructuringElement(cv.MORPH_RECT, dilation_size, dilation_anchor)
        image = cv.dilate(image, dilate_element)


        if config.blob_median_blur_size > 0:
            image = cv.medianBlur(image, config.blob_median_blur_size * 2 + 1);

        points = []
        theta = 0
        while theta <= math.pi:

            p = Point(config.blob_x, config.blob_y)
            diffx = math.cos(theta) * 0.01
            diffy = -1 * math.sin(theta) * 0.01

            # NOTE: may need to switch x/y here
            while image[int(p.y * rows(image)), int(p.x * cols(image))] < config.blob_num_points:
                p.x += diffx
                p.y += diffy

                top_y = 1 - config.blob_max_p_y
                if p.y > 1 or p.y < top_y or p.x > 1 or p.x < 0:
                    if p.x > 1: p.x = 1
                    if p.x < 0: p.x = 0
                    if p.y > 1: p.y = 1
                    if p.y < top_y: p.y = top_y
                    break

            if debug_image is not None:
                cv.circle(debug_image, 
                          (int(p.x * cols(debug_image)), int(p.y * rows(debug_image))),
                          5, (255, 0, 0), -1)

            points.append(p)

            theta += math.pi / config.blob_num_points

        center_p = Point(config.blob_x, config.blob_y)
        center_a = Point(0, 0)

        for p in points:
            diffx = center_p.x - p.x
            diffy = center_p.y - p.y

            center_a.x += p.x
            center_a.y += p.y

            length = math.sqrt(diffx * diffx + diffy * diffy);

            if length < .01: continue

            diffx /= length
            diffy /= length

            spring_force = -1 * config.blob_coeff * (length - config.blob_len)

            diffx *= spring_force
            diffy *= spring_force

            center_p.x = diffx
            center_p.y = diffy

        center_a.x /= len(points)
        center_a.y /= len(points)

        center_p.x = center_a.x + center_p.x
        center_p.y = center_a.y + center_p.y

        if debug_image is not None:
            cv.circle(debug_image, 
                      (int(center_p.x * cols(debug_image)), int(center_p.y * rows(debug_image))),
                      5, (0, 0, 255), -1)
            cv.circle(debug_image,
                      (int(config.blob_x * cols(debug_image)), int(config.blob_y * rows(debug_image))),
                      5, (0, 255, 0), -1)

        return center_p.x - config.blob_x
