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

# global variables
vel_msg = Twist()
bridge = CvBridge()
speed = 0
drive = False
velocity_pub = None
crop1 = 0
crop2 = 0

# dynamic reconfigure
def dyn_rcfg_cb(config, level):
    global speed, drive
    speed = config.speed
    drive = config.enable_drive
    return config

# compute lines and follow lane
def compute_lines(rows, image, crop1, crop2):
    # remove top of image
    image = image[rows // 9: int(rows * 6/7), :]

    # get image shape
    rows, cols, _ = image.shape
    # print(rows,cols)
    # 358,640
    # p1 = np.float32([[95, 0], [180, 0],  [0, 220], [360, 220]])           
    # p2 = np.float32([[0,  0], [360, 0],  [0, 220], [360, 220]])

    # matrix = cv.getPerspectiveTransform(p1, p2)
    # image = cv.warpPerspective(image, matrix, (360,220))

    #cv.imshow("Original image", image)

    # add median blur to emphasize white lines 
    image = cv.medianBlur(image, 5)

    #cv.imshow("median blur", image)

    mymask = np.zeros((rows, cols), dtype="uint8") # specifying dtype is critical
    myROI = [(cols // 2, 0), (0,crop1), (0, rows), (cols, rows), (cols, crop2), (cols //2, 0)]  # (x, y)
    cv.fillPoly(mymask, [np.array(myROI)], 255) # 255-white color
    # performing a bitwise_and with the image and the mask
    image  = cv.bitwise_and(image, image, mask=mymask)

    # mask white pixels
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(gray, 170, 255, cv.THRESH_BINARY)

    #cv.imshow("white thresh", thresh)

    # canny filtering to get edges
    edges = cv.Canny(thresh, 50, 150, 3)

    # cv.imshow("Canny", edges)

    # get list of lines
    lines = cv.HoughLinesP(edges, 1, np.pi / 180, threshold=20, minLineLength=20, maxLineGap=10)
    # rospy.loginfo(f"original - number of lines: {len(lines)}")
    
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
            if length < 50 and abs(slope) >= 0.4:
                # Extend the line by a factor of 1.5 times its original length
                extend_factor = 2
                x1_extended = int(x1 - (x2 - x1) * (extend_factor - 1))
                y1_extended = int(y1 - (y2 - y1) * (extend_factor - 1))
                x2_extended = int(x2 + (x2 - x1) * (extend_factor - 1))
                y2_extended = int(y2 + (y2 - y1) * (extend_factor - 1))
                
                cv.line(line_image, (x1_extended, y1_extended), (x2_extended, y2_extended), (255, 255, 255), 2)
            elif abs(slope) >= 0.4:
                cv.line(line_image, (x1, y1), (x2, y2), (255, 255, 255), 2)

    # cv.imshow("hough", line_image)

    # get white points
    points = np.column_stack(np.where(line_image > 0))   
    
    if len(points) == 0:
        print("No white pixels detected.")
        return image

    downsample_factor = int(len(points) / (0.01 * len(points)))
    points = points[::downsample_factor]

    dbscan = DBSCAN(eps=180, min_samples=1)
    clusters = dbscan.fit_predict(points)

    # get the two largest clusters 
    unique_labels, counts = np.unique(clusters, return_counts=True)
    sorted_clusters = sorted(list(zip(unique_labels, counts)), key=lambda x: -x[1])
    largest_labels = [sorted_clusters[i][0] for i in range(min(2, len(sorted_clusters)))]
    colors = [(0, 255, 0), (0, 0, 255)]
    label_to_color = {label: colors[i] for i, label in enumerate(largest_labels)}

    if len(largest_labels) == 0:
        return None, 0
    elif len(largest_labels) == 1:
        # image2 = image.copy()
        # lane_points = points[clusters == largest_labels[0]]
        # lane_points = lane_points[np.argsort(lane_points[:, 1])]

        # coefficients = np.polyfit(lane_points[:, 0], lane_points[:, 1], deg=2)
        # polynomial = np.poly1d(coefficients)

        # x_values = np.linspace(0, cols, num=100)
        # y_values = polynomial(x_values).astype(int)
        # for p in lane_points:
        #     cv.circle(image2, (int(p[1]), int(p[0])), 5, (0,255,0), 2)
        # for i in range(len(y_values) - 1):
        #     cv.line(image, (int(y_values[i]), int(x_values[i])), 
        #             (int(y_values[i+1]), int(x_values[i+1])), (0,255,0), 5)
        
        # centroid_x = np.mean(lane_points[:, 1]) 
        # if centroid_x < (cols // 2):
        #     cx = int(np.mean([centroid_x, cols]))
        # else:
        #     cx = int(np.mean([centroid_x, 0]))
        return None, 0

    else:
        lane_centroids = []
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
            lane_centroids.append(centroid_x)

        # get cx from the average of the two clusters' centroids
        if len(lane_centroids) == 2:
            cx = int(np.mean(lane_centroids))
        else:
            cx = cols // 2 

    # display image with lanes
    cv.circle(image, (cols//2,rows // 2), 5, (0, 0, 0), -1)
    cv.arrowedLine(image, (cols//2,rows // 2), (cx,rows // 2), (255, 255, 255), 3)
    # cv.imshow("Polynomial lanes", image)
    cv.imshow("Labeled points", image2)
    cv.waitKey(3) 
    return cx, (cx - (cols // 2))


def image_callback(ros_image):
    global bridge, vel_msg, speed, drive, velocity_pub, crop1, crop2
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    rows, cols, _ = cv_image.shape
    image = cv_image.copy()

    # compute lines and obtain cx
    cx, gap = compute_lines(rows, image, crop1, crop2)

    if gap < 0:
        crop2 = -gap
        crop1 = 0
    else:
        crop1 = gap
        crop2 = 0

    if cx:
        mid = cols / 2
        if drive:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = 0

        angular_threshold = 20
        if mid < cx - angular_threshold:
            vel_msg.angular.z = -abs(speed * ((mid - cx) / mid) ** 2)
        elif mid > cx + angular_threshold:
            vel_msg.angular.z = abs(speed * ((mid - cx) / mid)**2)
        else:
            vel_msg.angular.z = 0
        
        velocity_pub.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node('node1', anonymous=True)
    imgtopic = rospy.get_param("~imgtopic_name")
    rospy.Subscriber(imgtopic, Image, image_callback)
    velocity_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
    srv = Server(FollowLaneConfig, dyn_rcfg_cb)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
