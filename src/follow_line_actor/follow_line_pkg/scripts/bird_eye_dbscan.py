#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from follow_lane_pkg.cfg import FollowLaneConfig  # packageName.cfg
from geometry_msgs.msg import Twist
import cv2 as cv
import numpy as np
from sklearn.cluster import DBSCAN
import math

# global variables
vel_msg = Twist()
bridge = CvBridge()
speed = 0
drive = False
velocity_pub = None

# dynamic reconfigure
def dyn_rcfg_cb(config, level):
    global speed, drive
    speed = config.speed
    drive = config.enable_drive
    return config

# compute lines and follow lane
def compute_lines(rows, image):

    # remove top of image
    image = image[rows // 9: int(rows * 6/7), :]

    # get image shape
    rows, cols, _ = image.shape

    #birds eye view
    p1 = [0, 0]
    p2 = [cols//2, 0]
    p3 = [cols//2 - int(cols * 0.15), rows]
    p4 = [int(cols * 0.15), rows]
    
    per1 = np.float32([[0, 0], [cols, 0], [cols, rows], [0, rows]])
    per2 = np.float32([p1, p2, p3, p4])
    
    matrix = cv.getPerspectiveTransform(per1, per2)
    image = cv.warpPerspective(image, matrix, (cols//2, rows))
    rows, cols, _ = image.shape

    # add median blur to emphasize white lines 
    image = cv.medianBlur(image, 5)

    # mask white pixels
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(gray, 170, 255, cv.THRESH_BINARY)

    # canny filtering to get edges
    edges = cv.Canny(thresh, 50, 150, 3)

    # get list of lines
    lines = cv.HoughLinesP(edges, 1, np.pi / 180, threshold=20, minLineLength=20, maxLineGap=10)
    
    # empty image to draw filtered lines
    line_image = np.zeros_like(image)

    # filter out lines with a small slope and increase length of short ones
    if lines is not None:
        for line in lines:

            x1, y1, x2, y2 = line[0]
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            if x2 - x1 == 0:
                slope = float('inf')
            else:
                slope = (y2 - y1) / (x2 - x1)
            if length < 100 and abs(slope) >= 0.4:
                extend_factor = 2.5
                x1_extended = int(x1 - (x2 - x1) * (extend_factor - 1))
                y1_extended = int(y1 - (y2 - y1) * (extend_factor - 1))
                x2_extended = int(x2 + (x2 - x1) * (extend_factor - 1))
                y2_extended = int(y2 + (y2 - y1) * (extend_factor - 1))
                cv.line(line_image, (x1_extended, y1_extended), (x2_extended, y2_extended), (255, 255, 255), 2)
            elif abs(slope) >= 0.5:
                cv.line(line_image, (x1, y1), (x2, y2), (255, 255, 255), 2)

    cv.imshow("hough", line_image)

    # get white points
    points = np.column_stack(np.where(line_image > 0))   
    
    if len(points) == 0:
        print("No white pixels detected.")
        return image
    
    # downsample points for unsupervised learning
    downsample_factor = int(len(points) / (0.01 * len(points)))
    points = points[::downsample_factor]

    # find clusters using dbscan density based clustering
    dbscan = DBSCAN(eps=50, min_samples=3)
    clusters = dbscan.fit_predict(points)

    # get the two clusters closest to bottom with certain minimum size
    unique_labels, counts = np.unique(clusters, return_counts=True)
    valid_clusters = []

    for label in unique_labels:
        if label == -1:  
            continue

        cluster_points = points[clusters == label]

        if len(cluster_points) >= 10:
            centroid_y = np.max(cluster_points[:, 0])
            valid_clusters.append((label, centroid_y))

    sorted_clusters = sorted(valid_clusters, key=lambda x: x[1], reverse=True)[:2]
    largest_labels = [sorted_clusters[i][0] for i in range(min(2, len(sorted_clusters)))]
    colors = [(0, 255, 0), (0, 0, 255)]
    label_to_color = {label: colors[i] for i, label in enumerate(largest_labels)}

    # conditions
    if len(largest_labels) == 0:
        return None
    
    elif len(largest_labels) == 1:
        lane_points = points[clusters == largest_labels[0]]
        centroid_x = lane_points[np.argmax(lane_points[:, 0])][1]
        image2 = image.copy()
        for p in lane_points:
            cv.circle(image2, (int(p[1]), int(p[0])), 5, (0,255,0), 2)
        cv.imshow("Labeled points", image2)
        cv.waitKey(3) 
        if centroid_x < (cols // 2):
            cx = (cols // 2 + 100)
            print(cx)
            return cx
        else:
            cx = (cols // 2 - 100)
            print(cx)
            return cx

    else:
        # if two clusters, find center of both
        lane_centroids_x = []
        lane_centroids_y = []
        image2 = image.copy()
        for label in largest_labels:

            lane_points = points[clusters == label]
            lane_points = lane_points[np.argsort(lane_points[:, 1])]

            coefficients = np.polyfit(lane_points[:, 0], lane_points[:, 1], deg=2)
            polynomial = np.poly1d(coefficients)

            x_values = np.linspace(0, cols, num=100)
            y_values = polynomial(x_values).astype(int)
            
            for p in lane_points:
                cv.circle(image2, (int(p[1]), int(p[0])), 5, label_to_color[label], 2)
            for i in range(len(y_values) - 1):
                cv.line(image, (int(y_values[i]), int(x_values[i])), 
                        (int(y_values[i+1]), int(x_values[i+1])), label_to_color[label], 5)
            
            centroid_x = np.mean(lane_points[:, 1]) 
            centroid_y = np.mean(lane_points[:, 0]) 
            lane_centroids_x.append(centroid_x)
            lane_centroids_y.append(centroid_y)

        # get cx from the average of the two clusters' centroids
        cx = int(np.mean(lane_centroids_x))
        cy = int(np.mean(lane_centroids_y))

    cv.arrowedLine(image, (cols//2,rows-1), (cx,cy), (255, 255, 255), 2)
    cv.imshow("Polynomial lanes", image)
    cv.imshow("Labeled points", image2)
    cv.waitKey(3) 
    return cx, cy, cols, rows

# convert angle to yaw rate
def angle_to_yaw(angle_degrees, max_steer_angle=11.25, max_yaw_rate=1.0):

    angle_degrees = min(max_steer_angle, angle_degrees)
    angle_degrees = (angle_degrees + 180) % 360 - 180  # normalize the angle to the range [-180, 180]
    angle_radians = math.radians(angle_degrees)
    yaw_rate = max_yaw_rate * angle_radians / math.radians(max_steer_angle) # scale the angle to the maximum yaw rate
    return yaw_rate

# image callback
def image_callback(ros_image):
    global bridge, vel_msg, speed, drive, velocity_pub

    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    rows, cols, _ = cv_image.shape
    image = cv_image.copy()

    # compute lines and obtain ideal image center
    cx, cy, cols, rows = compute_lines(rows, image)

    # get turn angle
    mid_x = cols // 2
    angle = float(math.degrees(math.atan2(abs(mid_x - cx), abs(rows - cy)))) * 0.75

    # compute linear and angular speed
    if cx:
        if drive:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = 0

        angular_threshold = 20
        if mid_x < cx - angular_threshold:
            vel_msg.angular.z = -angle_to_yaw(angle)
        elif mid_x > cx + angular_threshold:
            vel_msg.angular.z = angle_to_yaw(angle)
        else:
            vel_msg.angular.z = 0
        velocity_pub.publish(vel_msg)
    else:
        vel_msg.angular.z = 0
        velocity_pub.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node('bird_eye_dbscan', anonymous=True)
    imgtopic = rospy.get_param("~imgtopic_name")
    rospy.Subscriber(imgtopic, Image, image_callback)
    velocity_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
    srv = Server(FollowLaneConfig, dyn_rcfg_cb)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass