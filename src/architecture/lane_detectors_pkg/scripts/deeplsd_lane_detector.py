#!/usr/bin/env python3
import math
import os
import threading

import cv2 as cv
import numpy as np
import rospy
import torch
from cv_bridge import CvBridge, CvBridgeError
from DeepLSD.deeplsd.models.deeplsd_inference import DeepLSD
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sklearn.cluster import DBSCAN

# from dynamic_reconfigure.server import Server
# from lane_detectors_pkg.cfg import ...  # packageName.cfg

class DeepLSDLaneDetector:
    """
    A lane dector using DeepLSD.
    """

    preprocessed_img_subscriber: rospy.Subscriber
    center_offset_publisher: rospy.Publisher

    offset_message: Twist

    # ------------------------------------------------
    # ------- Internal state-related functions -------
    # ------------------------------------------------
    def __init__(self):
        """
        Initializes the node.
        """
        # Node architecture
        rospy.init_node("template_lane_detector", anonymous=True)

        self.preprocessed_img_subscriber = rospy.Subscriber(
            rospy.get_param("~img_in_topic"), Image, self._find_lane
        )
        self.center_offset_publisher = rospy.Publisher(
            rospy.get_param("~desired_twist_out_topic"), Twist, queue_size=1
        )

        self.offset_message = Twist()
        self.rosimg_cv_bridge = CvBridge()
        self.last_processed_time = 0

        self.desired_twist_x = 0
        self.desired_twist_y = 0

        self.image_lock = threading.Lock()
        self.ros_image = None

        self.inference_thread = threading.Thread(target=self._run_inference_loop)
        self.inference_thread.daemon = True
        self.inference_thread.start()

        self.publisher_thread = threading.Thread(target=self.publisher)
        self.publisher_thread.daemon = True
        self.publisher_thread.start()

    # ---------------------------------------
    # ----------- Lane detection ------------
    # ---------------------------------------

    def _run_inference_loop(self):
        rate = rospy.Rate(5)  # 5 Hz inference rate
        while not rospy.is_shutdown():
            with self.image_lock:
                if self.ros_image is not None:
                    self.desired_twist_x, self.desired_twist_y = self._perform_inference(self.ros_image)
            rate.sleep()

    def _perform_inference(self, ros_image):
        try:
            grayscale_cv_image = self.rosimg_cv_bridge.imgmsg_to_cv2(ros_image)
            grayscale_cv_image = grayscale_cv_image.copy()
            ros_image = cv.cvtColor(grayscale_cv_image, cv.COLOR_GRAY2BGR)
            return self.inference_deeplsd(ros_image)
        except CvBridgeError as e:
            rospy.logerr(f"DeepLSDLaneDetector:109 - CvBridge Error: {e}")
            return 0, 0

    def publisher(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            self.offset_message.angular.x = self.desired_twist_x
            self.offset_message.angular.y = self.desired_twist_y
            self.offset_message.angular.z = 1
            self.center_offset_publisher.publish(self.offset_message)
            rate.sleep()

    def _find_lane(self, ros_image: Image) -> None:
        with self.image_lock:
            self.ros_image = ros_image
        
    def inference_deeplsd(self, ros_image):
    
        rows, cols, _ = ros_image.shape

        # draw horizontal lines to improve curve detection using DeepLSD
        line_spacing = rows // (10 + 1)
        for i in range(10):
            y = line_spacing * (i + 1)
            cv.line(ros_image, (0, y), (cols - 1, y), (0, 0, 0), 5)
        image2 = ros_image.copy()

        # emphasize lines using dilate
        kernel = np.ones((5, 5), np.uint8)
        dilated = cv.dilate(ros_image, kernel, iterations=1)

        # get white mask
        gray_img = cv.cvtColor(dilated, cv.COLOR_RGB2GRAY) 
        _, image = cv.threshold(gray_img, 170, 255, cv.THRESH_BINARY)

        # smoothen contours
        gray_img = cv.GaussianBlur(image, (5, 5), 0) 

        # INFERENCE DEEPLSD
        inputs = {'image': torch.tensor(gray_img, dtype=torch.float, device=device)[None, None] / 255.}
        with torch.no_grad():
            out = net(inputs)
            pred_lines = out['lines'][0]

        # Filter lines by slope and length
        filtered_lines = []
        for line in pred_lines:
            if len(line) == 2:
                x1, y1 = line[0]
                x2, y2 = line[1]
                
                # Calculate slope
                if x2 != x1:
                    slope = (y2 - y1) / (x2 - x1)
                    
                    # Filter based on slope and length
                    line_length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    min_slope = 0.3
                    min_length = 10
                    
                    if abs(slope) > min_slope and line_length > min_length:

                        extend_factor = 2.0
                        x1_extended = int(x1 - (x2 - x1) * (extend_factor))
                        y1_extended = int(y1 - (y2 - y1) * (extend_factor))
                        x2_extended = int(x2 + (x2 - x1) * (extend_factor))
                        y2_extended = int(y2 + (y2 - y1) * (extend_factor))
                        filtered_lines.append([(x1_extended, y1_extended), (x2_extended, y2_extended)])
                        cv.line(image2, (int(x1_extended), int(y1_extended)), (int(x2_extended), int(y2_extended)), (0, 0, 255), 2)


        def get_red_pixels(image):
            # Create a mask for red color in BGR
            red_mask = cv.inRange(image, (0, 0, 255), (0, 0, 255))
            
            # Extract red pixels' coordinates
            red_pixels = np.column_stack(np.where(red_mask > 0))
            return red_pixels
    
        points = get_red_pixels(image2)
        downsample_factor = int(len(points) / (0.02 * len(points)))
        points = points[::downsample_factor]   

        # UNSUPERVISED LEARNING USING DBSCAN
        # find clusters using dbscan density based clustering
        dbscan = DBSCAN(eps=100, min_samples=2)
        clusters = dbscan.fit_predict(points)

        # get the two clusters closest to bottom with certain minimum size
        unique_labels, counts = np.unique(clusters, return_counts=True)
        valid_clusters = []

        for label in unique_labels:
            if label == -1:  
                continue

            cluster_points = points[clusters == label]

            if len(cluster_points) >= 8:
                centroid_y = np.max(cluster_points[:, 0])
                valid_clusters.append((label, centroid_y))

        sorted_clusters = sorted(valid_clusters, key=lambda x: x[1], reverse=True)[:2]
        largest_labels = [sorted_clusters[i][0] for i in range(min(2, len(sorted_clusters)))]
        colors = [(0, 255, 0), (0, 0, 255)]
        label_to_color = {label: colors[i] for i, label in enumerate(largest_labels)}

        # conditions
        if len(largest_labels) == 0:
            return None
        
        # if only one cluster detected
        elif len(largest_labels) == 1:
            lane_points = points[clusters == largest_labels[0]]
            centroid_x = np.mean(lane_points[:, 1])  # Mean of x-coordinates
            centroid_y = np.mean(lane_points[:, 0])  # Mean of y-coordinates
            for p in lane_points:
                cv.circle(image2, (int(p[1]), int(p[0])), 5, (0, 255, 0), 2)
            if centroid_x < (cols // 2):
                desired_midpoint_x = (cols // 2 + 100)
            else:
                desired_midpoint_x = (cols // 2 - 100)
            desired_midpoint_y = rows // 2

        # if two clusters, find center of both
        else:
            lane_centroids_x = []
            lane_centroids_y = []
            for label in largest_labels:

                lane_points = points[clusters == label]
                lane_points = lane_points[np.argsort(lane_points[:, 1])]
                
                for p in lane_points:
                    cv.circle(image2, (int(p[1]), int(p[0])), 5, label_to_color[label], 2)
                
                centroid_x = np.mean(lane_points[:, 1]) 
                centroid_y = np.mean(lane_points[:, 0]) 
                lane_centroids_x.append(centroid_x)
                lane_centroids_y.append(centroid_y)

            # get cx from the average of the two clusters' centroids
            desired_midpoint_x = int(np.mean(lane_centroids_x))
            desired_midpoint_y = int(np.mean(lane_centroids_y))

        cv.arrowedLine(image2, (cols//2,rows-1), (desired_midpoint_x,desired_midpoint_y), (255, 255, 255), 2)
        cv.imshow("lanes", image2)
        cv.waitKey(3)

        image_midpoint_x = cols // 2
        self.desired_twist_x = (desired_midpoint_x - image_midpoint_x) / image_midpoint_x
        self.desired_twist_y = (rows - desired_midpoint_y) / rows
        return self.desired_twist_x, self.desired_twist_y

if __name__ == "__main__":
    try:
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        conf = {
            'detect_lines': True,
            'line_detection_params': {
                'merge': False,
                'filtering': True,
                'grad_thresh': 3,
                'grad_nfa': True,
            }
        }

        home_dir = os.environ['HOME']
        potential_dir = os.path.join(home_dir, 'reu_ws/src')
        if os.path.isdir(potential_dir):
            ckpt = os.path.join(home_dir, 'reu_ws/src/architecture/lane_detectors_pkg/scripts/weights/deeplsd_md.tar')
        else:
            potential_dir = os.path.join(home_dir, 'REU_ws/src')
            if os.path.isdir(potential_dir):
                ckpt = os.path.join(home_dir, 'REU_ws/src/architecture/lane_detectors_pkg/scripts/weights/deeplsd_md.tar')
            else:
                ckpt = os.path.join(home_dir, 'reu_ws/REU-2024/src/architecture/lane_detectors_pkg/scripts/weights/deeplsd_md.tar')
        # Marcial note: Yes, this code is garbage. I'm tired. Fix it yourself if you hate it so much. (Not you Benat, ily <3)
        

        ckpt = torch.load(str(ckpt), map_location='cpu')
        net = DeepLSD(conf)
        net.load_state_dict(ckpt['model'])
        net = net.to(device).eval()
        detector = DeepLSDLaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
