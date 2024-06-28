#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from follow_line_pkg.cfg import FollowLineConfig  # packageName.cfg
from geometry_msgs.msg import Twist
import cv2 as cv
import numpy as np
from sklearn.cluster import DBSCAN
# from scipy.cluster.hierarchy import linkage, fcluster
# from sklearn.cluster import KMeans

# global variables
vel_msg = Twist()
bridge = CvBridge()
speed = 0
drive = False
velocity_pub = None
empty_msg = Empty()


# dynamic reconfigure
def dyn_rcfg_cb(config, level):
  global thresh, speed, drive
  thresh = config.thresh
  speed = config.speed
  drive = config.enable_drive
  return config # must return config

# compute lines and follow lane
def compute_lines(rows, image):

    # remove top of image
    image = image[rows // 7:, :]

    # add median blur to emphasize white lines 
    image = cv.medianBlur(image, 5)

    # get image shape
    rows, cols, _ = image.shape

    # mask white pixels
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(gray, 170, 255, cv.THRESH_BINARY)

    # canny filtering to get edges
    edges = cv.Canny(thresh, 50, 150, 3)

    # make lines thicker
    element = cv.getStructuringElement(cv.MORPH_RECT, (5,5))
    image = cv.dilate(image, element)

    # get list of lines
    lines = cv.HoughLinesP(edges, 1, np.pi / 180, threshold=20, minLineLength=40, maxLineGap=10)
    
    # empty image to draw filtered lines
    line_image = np.zeros_like(image)

    # filter out lines with a small slope
    if lines is not None:
        for line in lines:

            x1, y1, x2, y2 = line[0]
            dx = x2 - x1
            dy = y2 - y1
            if dx == 0: 
                slope = float('inf')
            else:
                slope = dy / dx

            if abs(slope) >= 0.5:
                cv.line(line_image, (x1, y1), (x2, y2), (255, 255, 255), 2)

    # get white points
    points = np.column_stack(np.where(line_image > 0))   
    
    if len(points) == 0:
        print("No white pixels detected.")
        return image

    # downsample the points uniformly for clustering
    downsample_factor = 10  
    points = points[::downsample_factor]

    # perform density based clustering
    dbscan = DBSCAN(eps=200, min_samples=10)
    clusters = dbscan.fit_predict(points)

    # get the two largest clusters 
    unique_labels, counts = np.unique(clusters, return_counts=True)
    sorted_clusters = sorted(list(zip(unique_labels, counts)), key=lambda x: -x[1])
    largest_labels = [sorted_clusters[i][0] for i in range(min(2, len(sorted_clusters)))]
    colors = [(0, 255, 0), (0, 0, 255)]
    label_to_color = {label: colors[i] for i, label in enumerate(largest_labels)}

    lane_centroids = []
    mass = []

    # for each cluster fit a 2nd degree polynomial 
    for label in largest_labels:

        lane_points = points[clusters == label]
        lane_points = lane_points[np.argsort(lane_points[:, 1])]

        coefficients = np.polyfit(lane_points[:, 0], lane_points[:, 1], deg=2)
        polynomial = np.poly1d(coefficients)

        x_values = np.linspace(np.min(lane_points[:, 0]), np.max(lane_points[:, 0]), num=100)
        y_values = polynomial(x_values).astype(int)
        for i in range(len(y_values) - 1):
            cv.line(image, (int(y_values[i]), int(x_values[i])), 
                    (int(y_values[i+1]), int(x_values[i+1])), label_to_color[label], 5)
        
        centroid_x = np.mean(lane_points[:, 1]) 
        lane_centroids.append(centroid_x)
        mass.append(len(lane_points))

    # get cx from the average of the two clusters' centroids
    if len(lane_centroids) == 2:
        cx = int(np.mean(lane_centroids))
    else:
        cx = cols // 2 

    # display image with lanes
    cv.imshow("White Points", image)
    cv.waitKey(3) 
    return cx


def image_callback(ros_image):
    global bridge, vel_msg, speed, drive, velocity_pub
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    rows, cols, channels = cv_image.shape
    image = cv_image.copy()

    # compute lines and obtain cx
    cx = compute_lines(rows, image)

    # compute linear x
    mid = cols / 2
    error = cx - mid
    if drive:
        vel_msg.linear.x = 3.0  
    else:
        vel_msg.linear.x = 0

    # compute angular z
    c = 0.012
    vel_msg.angular.z = -c * error

    # publish changes
    velocity_pub.publish(vel_msg)
    enable_pub.publish(empty_msg)

# main method
if __name__ == '__main__':
  rospy.init_node('advanced_follow_lane', anonymous=True) # initialize node
  imgtopic = rospy.get_param("~imgtopic_name") # private name
  rospy.Subscriber(imgtopic, Image, image_callback) # subscribe to image (so that image_callback can be called every time an image is published)
  enable_pub = rospy.Publisher('/vehicle/enable', Empty, queue_size=1)      # publish to enable (to enable the robot)
  velocity_pub = rospy.Publisher('/vehicle/cmd_vel', Twist, queue_size=1)   # publish to cmd_vel (to move the robot)
  srv = Server(FollowLineConfig, dyn_rcfg_cb) # create dynamic reconfigure server that calls dyn_rcfg_cb function every time a parameter is changed

  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass