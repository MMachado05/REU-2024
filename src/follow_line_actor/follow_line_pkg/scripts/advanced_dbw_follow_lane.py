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
import time

# dbw commands
import math
from dataspeed_ulc_msgs.msg import UlcCmd # Drive by wire UL 
from dbw_polaris_msgs.msg import SteeringCmd # drive by wire native messages


# global variables
vel_msg = Twist()
bridge = CvBridge()
# speed = 0
# drive = False
velocity_pub = None
empty_msg = Empty()
crop1 = 0
crop2 = 0

start_time = None
start_time2 = None



# dynamic reconfigure
def dyn_rcfg_cb(config, level):
  global thresh, speed, drive, ly, uy, ls, lv, us, uv
  thresh = config.thresh
  speed = config.speed
  drive = config.enable_drive
  ly = config.lower_yellow
  uy = config.upper_yellow
  ls = config.ls
  lv = config.lv
  us = config.us
  uv = config.uv
  return config # must return config

# compute lines and follow lane
def compute_lines(rows, cols, image, crop1, crop2):


    # add median blur to emphasize white lines 
    image = cv.medianBlur(image, 5)
    mymask = np.zeros((rows, cols), dtype="uint8") # specifying dtype is critical
    myROI = [(cols // 2, 0), (0,crop1), (0, rows), (cols, rows), (cols, crop2), (cols //2, 0)]  # (x, y)
    cv.fillPoly(mymask, [np.array(myROI)], 255) # 255-white color
    # performing a bitwise_and with the image and the mask
    image  = cv.bitwise_and(image, image, mask=mymask)

    # mask white pixels
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(gray, 210, 255, cv.THRESH_BINARY)

    # canny filtering to get edges
    edges = cv.Canny(thresh, 50, 150, 3)

    # get list of lines
    lines = cv.HoughLinesP(edges, 3, np.pi / 180, threshold=20, minLineLength=10, maxLineGap=10)
    
    # empty image to draw filtered lines
    line_image = np.zeros_like(image)

    # filter out lines with a small slope
    if lines is not None:
        for line in lines:

            # x1, y1, x2, y2 = line[0]
            # dx = x2 - x1
            # dy = y2 - y1
            # if dx == 0: 
            #     slope = float('inf')
            # else:
            #     slope = dy / dx

            # if abs(slope) >= 0.4:
            #     cv.line(line_image, (x1, y1), (x2, y2), (255, 255, 255), 2)
            x1, y1, x2, y2 = line[0]
            
            # Calculate length of the line segment
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            # Calculate slope of the line
            if x2 - x1 == 0:
                slope = float('inf')
            else:
                slope = (y2 - y1) / (x2 - x1)
            
            # Extend the line if its length is shorter than 40 pixels and slope is >= 0.4 or <= -0.4
            if length < 50 and abs(slope) >= 0.5:
                # Extend the line by a factor of 1.5 times its original length
                extend_factor = 2
                x1_extended = int(x1 - (x2 - x1) * (extend_factor - 1))
                y1_extended = int(y1 - (y2 - y1) * (extend_factor - 1))
                x2_extended = int(x2 + (x2 - x1) * (extend_factor - 1))
                y2_extended = int(y2 + (y2 - y1) * (extend_factor - 1))
                
                cv.line(line_image, (x1_extended, y1_extended), (x2_extended, y2_extended), (255, 255, 255), 2)
            elif abs(slope) >= 0.4:
                cv.line(line_image, (x1, y1), (x2, y2), (255, 255, 255), 2)

    # make lines thicker
    #element = cv.getStructuringElement(cv.MORPH_RECT, (5,5))
    #line_image = cv.dilate(line_image, element)
    cv.imshow("canny", line_image)

    # get white points
    points = np.column_stack(np.where(line_image > 0))   
    
    if len(points) == 0:
        print("No white pixels detected.")
        return None, 0

    # downsample the points uniformly for clustering
    # downsample_factor = 10
    # points = points[::downsample_factor]
    downsample_factor = int(len(points) / (0.01 * len(points)))
    points = points[::downsample_factor]

    # perform density based clustering
    dbscan = DBSCAN(eps=100, min_samples=3)
    clusters = dbscan.fit_predict(points)

    # get the two largest clusters 
    unique_labels, counts = np.unique(clusters, return_counts=True)
    sorted_clusters = sorted(list(zip(unique_labels, counts)), key=lambda x: -x[1])
    largest_labels = [sorted_clusters[i][0] for i in range(min(2, len(sorted_clusters)))]
    colors = [(0, 255, 0), (0, 0, 255)]
    label_to_color = {label: colors[i] for i, label in enumerate(largest_labels)}

    if len(unique_labels) < 2:
        print("a")
        return None, 0
    
    lane_centroids = []
    # for each cluster fit a 2nd degree polynomial 
    for label in largest_labels:

        lane_points = points[clusters == label]
        lane_points = lane_points[np.argsort(lane_points[:, 1])]

        coefficients = np.polyfit(lane_points[:, 0], lane_points[:, 1], deg=2)
        polynomial = np.poly1d(coefficients)

        x_values = np.linspace(0, cols, num=100)
        y_values = polynomial(x_values).astype(int)
        # for p in lane_points:
        #     cv.circle(image, (int(p[1]), int(p[0])), 5, label_to_color[label], -1)
        for i in range(len(y_values) - 1):
            cv.line(image, (int(y_values[i]), int(x_values[i])), 
                    (int(y_values[i+1]), int(x_values[i+1])), label_to_color[label], 5)
        
        centroid_x = np.mean(lane_points[:, 1]) 
        lane_centroids.append(centroid_x)

    # get cx from the average of the two clusters' centroids
    if len(lane_centroids) == 2:
        cx = int(np.mean(lane_centroids))
        cv.circle(image, (cols // 2, rows // 2), 5, (0, 0, 0), -1)
        cv.arrowedLine(image, (cols // 2, rows // 2), (cx, rows // 2), (255, 255, 255), 3)
    else:
        cx = cols // 2 

    # display image with lanes
    cv.imshow("White Points", image)
    cv.waitKey(3)

    
    return cx, (cx - (cols // 2))


def image_callback(ros_image):
    global bridge, vel_msg, speed, drive, velocity_pub, crop1,crop2, start_time, start_time2
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return
    image2 = cv_image.copy()
    # remove top of image
    rows1, cols1, _ = cv_image.shape
    cv_image = cv_image[rows1 // 2:,:]
    rows, cols, _ = cv_image.shape

    mymask = np.zeros((rows, cols), dtype="uint8")
    myROI = [(cols // 2, 0), (0,(rows // 4)),(0,rows),(cols, rows), (cols, (rows //4)), (cols, 0)]
    cv.fillPoly(mymask, [np.array(myROI)], 255)
    cv_image = cv.bitwise_and(cv_image, cv_image, mask = mymask)

    image = cv_image.copy()

    
    image2 = image2[int(rows1*3/5):int(rows1*4/5), int(cols1 * 1 / 3): int(cols1 * 2/3)]
    rows2, cols2, _ = image2.shape
    # yellow mask
    hsv = cv.cvtColor(image2,cv.COLOR_BGR2HSV)
    lower_yellow = np.array([ly, ls, lv]) 
    upper_yellow = np.array([uy, us, uv])
    yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)

    # check for intersection
    num_yellow_pix = cv.countNonZero(yellow_mask)
    yellow_pct = (100 * num_yellow_pix) / (rows2 * cols2)
    print(yellow_pct)
        # show image
    cv.imshow("yellow", yellow_mask)
    cv.waitKey(3)    

    # compute lines and obtain cx
    cx, gap = compute_lines(rows, cols, image, crop1, crop2)
    if gap < 0:
        crop2 = -gap
        crop1 = 0
    else:
        crop1 = gap
        crop2 = 0

    angle = float(math.degrees(abs(gap)/(rows//2)) / 2)
    target_speed = speed #mph
    

    if cx:
        mid = cols / 2
        if drive:
            if start_time2 is not None:
                current_time = time.time()
                if (current_time - start_time2) < 20:
                    publish_ulc_speed(speed)
                else:
                    start_time2 = None
            elif yellow_pct > 5 and start_time == None:
                start_time = time.time()
            elif start_time is not None:
                current_time = time.time()
                if (current_time - start_time) < 5:
                    publish_ulc_speed(0)
                else:
                    start_time = None
                    start_time2 = time.time()

                # time.sleep(10)
                # start_time = time.time()
                # while True:
                #     publish_ulc_speed(target_speed)
                #     currrent_time = time.time()
                #     if (current_time - start_time) > 4:
                #         break
            else:
                publish_ulc_speed(target_speed)
        else:
            publish_ulc_speed(0)

        angular_threshold = 20
        if mid < cx - angular_threshold:
            publish_steering(-angle)
        elif mid > cx + angular_threshold:
            publish_steering(angle)
        else:
            publish_steering(0)
    enable_dbw()
    
def publish_ulc_speed(speed: float) -> None:
    """Publish requested speed to the vehicle using ULC message."""

    ulc_cmd = UlcCmd()
    ulc_cmd.enable_pedals = True
    ulc_cmd.enable_steering = False  # NOTE: Steering control via ULC is not used here
    ulc_cmd.enable_shifting = True
    ulc_cmd.shift_from_park = True

    ulc_cmd.linear_velocity = round((speed / 2.237), 3)  # Convert mph to m/s
    ulc_cmd.linear_accel = 0.0
    ulc_cmd.linear_decel = 0.0
    ulc_cmd.jerk_limit_throttle = 0.0
    ulc_cmd.jerk_limit_brake = 0.0

    ulc_cmd.pedals_mode = 0  # Speed mode
    # ---------------------------------------------------------------------
    pub_ulc.publish(ulc_cmd)


def publish_steering(requested_road_angle: float = None) -> None:
    """Publish requested steering to the vehicle.
    Input can be desired degree road angle (-37 to 37) or steering angle (-600 to 600)"""

    if requested_road_angle is None:
        rospy.logerr("publish_steering called with no steering angle provided")
        requested_road_angle = 0

    if requested_road_angle is not None:
        if requested_road_angle < 0:
            requested_road_angle = max(requested_road_angle, -37) * 16.2
        else:
            requested_road_angle = min(requested_road_angle, 37) * 16.2

    # Make steering message -----------------------------------------------
    msg_steering = SteeringCmd()
    msg_steering.steering_wheel_angle_cmd = math.radians(requested_road_angle)
    # NOTE: (-600deg to 600deg converted to radians)
    msg_steering.enable = True  # Enable Steering, required 'True' for control via ROS

    # Do NOT use these without completely understanding how they work on the hardware level:
    msg_steering.cmd_type = SteeringCmd.CMD_ANGLE  # CAUTION: Torque mode disables lateral acceleration limits
    # Use angle velocity to control rate. Lock to lock = 1200deg i.e. 300deg/s will be 4secs lock to lock
    msg_steering.steering_wheel_angle_velocity = math.radians(300)  # deg/s -> rad/s
    msg_steering.steering_wheel_torque_cmd = 0.0  # Nm
    msg_steering.clear = False
    msg_steering.ignore = False
    msg_steering.calibrate = False
    msg_steering.quiet = False
    msg_steering.count = 0
    # ---------------------------------------------------------------------
    pub_steering.publish(msg_steering)


def enable_dbw() -> None:
    """Enable vehicle control using ROS messages"""
    msg = Empty()
    pub_enable_cmd.publish(msg)



# main method
if __name__ == '__main__':
  rospy.init_node('advanced__dbw_follow_lane', anonymous=True) # initialize node
  imgtopic = rospy.get_param("~imgtopic_name") # private name
  rospy.Subscriber(imgtopic, Image, image_callback) # subscribe to image (so that image_callback can be called every time an image is published)
  pub_ulc = rospy.Publisher("/vehicle/ulc_cmd", UlcCmd, queue_size=1)
  pub_steering = rospy.Publisher("/vehicle/steering_cmd", SteeringCmd, queue_size=1)
  pub_enable_cmd = rospy.Publisher("/vehicle/enable", Empty, queue_size=1)
  srv = Server(FollowLineConfig, dyn_rcfg_cb) # create dynamic reconfigure server that calls dyn_rcfg_cb function every time a parameter is changed
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass