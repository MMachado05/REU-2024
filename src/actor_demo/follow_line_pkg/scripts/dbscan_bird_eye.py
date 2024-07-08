#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Float64
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from follow_line_pkg.cfg import FollowLineConfig  # packageName.cfg
from geometry_msgs.msg import Twist
import cv2 as cv
import numpy as np
from sklearn.cluster import DBSCAN
import time

# dbw commands
import math

# global variables
bridge = CvBridge()
thresh1 = 255
crop1 = 0
crop2 = 0
bird1 = 0.1

# dynamic reconfigure
def dyn_rcfg_cb(config, level):
  global thresh1, speed, drive, bird1
  thresh1 = config.thresh
  speed = config.speed
  drive = config.enable_drive
  bird1 = config.bird1
  return config # must return config

# compute lines and follow lane
def compute_lines(rows, cols, image, crop1, crop2):
    global thresh1

    # add median blur to emphasize white lines 
    image = cv.medianBlur(image, 5)
    mymask = np.zeros((rows, cols), dtype="uint8") # specifying dtype is critical
    myROI = [(cols // 2, 0), (0,crop1), (0, rows), (cols, rows), (cols, crop2), (cols //2, 0)]  # (x, y)
    cv.fillPoly(mymask, [np.array(myROI)], 255) # 255-white color
    # performing a bitwise_and with the image and the mask
    image  = cv.bitwise_and(image, image, mask=mymask)

    # mask white pixels
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(gray, thresh1, 255, cv.THRESH_BINARY)

    # canny filtering to get edges
    edges = cv.Canny(thresh, 50, 150, 3)

    # get list of lines
    lines = cv.HoughLinesP(edges, 3, np.pi / 180, threshold=20, minLineLength=10, maxLineGap=10)
    
    # empty image to draw filtered lines
    line_image = np.zeros_like(image)

    # filter out lines with a small slope
    if lines is not None:
        for line in lines:

            x1, y1, x2, y2 = line[0]
            
            # Calculate length of the line segment
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            # Calculate slope of the line
            if x2 - x1 == 0:
                slope = float('inf')
            else:
                slope = (y2 - y1) / (x2 - x1)
            
            # Extend the line if its length is shorter than 40 pixels and slope is >= 0.4 or <= -0.4
            if length < 100 and abs(slope) >= 0.5: # 50, 0.5
                # Extend the line by a factor of 1.5 times its original length
                extend_factor = 3 # 2
                x1_extended = int(x1 - (x2 - x1) * (extend_factor - 1))
                y1_extended = int(y1 - (y2 - y1) * (extend_factor - 1))
                x2_extended = int(x2 + (x2 - x1) * (extend_factor - 1))
                y2_extended = int(y2 + (y2 - y1) * (extend_factor - 1))
                
                cv.line(line_image, (x1_extended, y1_extended), (x2_extended, y2_extended), (255, 255, 255), 2)
            elif abs(slope) >= 0.5:
                cv.line(line_image, (x1, y1), (x2, y2), (255, 255, 255), 2)

    # make lines thicker
    #element = cv.getStructuringElement(cv.MORPH_RECT, (5,5))
    #line_image = cv.dilate(line_image, element)
    cv.imshow("canny", line_image)

    # get white points
    points = np.column_stack(np.where(line_image > 0))   
    
    if len(points) == 0:
        print("No white pixels detected.")
        return None, None, 1, 1

    # downsample the points uniformly for clustering
    # downsample_factor = 10
    # points = points[::downsample_factor]
    downsample_factor = 100
    points = points[::downsample_factor]

    # perform density based clustering
    dbscan = DBSCAN(eps=70, min_samples=3) # 100
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
        return None, None, 1, 1
    
    # if only one cluster
    elif len(largest_labels) == 1:
        lane_points = points[clusters == largest_labels[0]]
        centroid_x = lane_points[np.argmax(lane_points[:, 0])][1]
        image2 = image.copy()
        for p in lane_points:
            cv.circle(image2, (int(p[1]), int(p[0])), 5, (0,255,0), 2)
        cv.imshow("Labeled points", image2)
        cv.waitKey(3) 
        cy = rows // 2
        if centroid_x < (cols // 2):
            cx = (cols // 2 + 50)

            print(cx)
            return cx, cy, cols, rows
        else:
            cx = (cols // 2 - 50)
            print(cx)
            return cx, cy, cols, rows

    else:
        # if two clusters, find center of both
        lane_centroids_x = []
        lane_centroids_y = []
        image2 = image.copy()
        for label in largest_labels:

            lane_points = points[clusters == label]
            lane_points = lane_points[np.argsort(lane_points[:, 1])]

            coefficients = np.polyfit(lane_points[:, 0], lane_points[:, 1], deg=1)
            polynomial = np.poly1d(coefficients)

            x_values = np.linspace(0, rows, num=100)
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

def change_perspective(img):
    global bird1
    rows, cols, _ = img.shape
    
    p1 = [0, 0]
    p2 = [cols//2, 0]
    p3 = [cols//2 - int(cols * bird1), rows]
    p4 = [int(cols * bird1), rows]
    
    per1 = np.float32([[0, 0], [cols, 0], [cols, rows], [0, rows]])
    per2 = np.float32([p1, p2, p3, p4])
    
    matrix = cv.getPerspectiveTransform(per1, per2)
    result = cv.warpPerspective(img, matrix, (cols//2, rows))
    return result


def image_callback(ros_image):
    global bridge, speed, drive, crop1, crop2
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return
    image2 = cv_image.copy()
    # remove top of image
    rows1, cols1, _ = cv_image.shape
    cv_image = cv_image[rows1//2:, :] #[int(rows1 /2):,:]
    cv_image = change_perspective(cv_image)
    rows1, cols1, _ = cv_image.shape
    cv_image = cv_image[rows1//2:, :]
    rows, cols, _ = cv_image.shape

    # mymask = np.zeros((rows, cols), dtype="uint8")
    # myROI = [(cols // 2, 0), (0,(rows // 4)),(0,rows),(cols, rows), (cols, (rows //4)), (cols, 0)]
    # cv.fillPoly(mymask, [np.array(myROI)], 255)
    # cv_image = cv.bitwise_and(cv_image, cv_image, mask = mymask)

    image = cv_image.copy()
  

    # compute lines and obtain cx
    cx, cy, cols, rows = compute_lines(rows, cols, image, crop1, crop2)
    # if cx < 0:
    #     crop2 = abs(cx) +120
    #     crop1 = 0
    # else:
    #     crop1 = abs(cx)+120
    #     crop2 = 0

    # mid_x = cols // 2
    # angle = float(math.degrees(math.atan2(abs(mid_x - cx), abs(rows - cy))) / 2) 
    #angle = float(math.degrees(abs(gap)/(rows//2)) /2)
    target_speed = speed #mph

    if cx:
        mid = cols / 2
        angle = float(math.degrees(math.atan2(abs(mid - cx), abs(rows - cy))) / 2) 
  
        angular_threshold = 20
        if mid < cx - angular_threshold:
            pub_angle.publish(-angle)
        elif mid > cx + angular_threshold:
            pub_angle.publish(angle)
        else:
            pub_angle.publish(0.0)



# main method
if __name__ == '__main__':
    rospy.init_node('advanced_dbw_follow_lane', anonymous=True) # initialize node
    
    print('running follow lane node')
    
    imgtopic = rospy.get_param("~imgtopic_name") # private name
    rospy.Subscriber(imgtopic, Image, image_callback) # subscribe to image (so that image_callback can be called every time an image is published)

    pub_angle = rospy.Publisher("angle", Float64, queue_size=1)

    srv = Server(FollowLineConfig, dyn_rcfg_cb) # create dynamic reconfigure server that calls dyn_rcfg_cb function every time a parameter is changed
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
