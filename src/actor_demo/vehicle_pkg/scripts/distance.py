#!/usr/bin/env python3

import rospy
import math
import numpy as np
import yaml
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from sklearn.metrics.pairwise import haversine_distances

class Break(Exception): pass

def gps_position_cb(msg):
    global gps_latitude, gps_longitude

    gps_latitude = msg.latitude
    gps_longitude = msg.longitude

# compute distance to intersection and adjust speed if needed
def compute_distance():
    global intersections, waypoints

    # find the closest waypoint to the current position
    closest_idx = -1
    min_distance = float('inf')
    for i, waypoint in enumerate(waypoints):
        dist = haversine_distances((gps_latitude['x'] - waypoint[0])**2 + (gps_longitude['y'] - waypoint[1])**2)
        if dist < min_distance:
            min_distance = dist
            closest_idx = i

    # # sum the total distance to the next intersection
    # total_distance = 0.0
    # try:
    #     for i in range(closest_idx, len(waypoints) - 1):
    #         total_distance += math.sqrt((waypoints[i][0] - waypoints[i + 1][0])**2 + ((waypoints[i][1] - waypoints[i + 1][1])**2))
    #         for int in intersections:
    #             if waypoints[i + 1] == int[:2]:
    #                 direction = int[2]
    #                 #print(f"{int[3]} light")
    #                 raise Break
    # except Break:
    #     pass
              
    # if direction == 0:
    #     stop = stop_horiz
    # else:
    #     stop = not stop_horiz
    
    # t = total_distance / vel # time left to arrive at intersection with current speed

    # if stop: # light is red
    #     if t < 10 and vel == 5.0: # if time to arrive less than changing to green light
    #         return float(total_distance / 10) - 0.5 # adjust speed
    #     else:
    #         return 5.0 # else keep speed

    # else: # light is green
    #     # if t > 10 and t < 20 and vel == 5.0:
    #     #     return float(total_distance / 20) 
    #     return 5.0

# main method
if __name__ == '__main__':
    rospy.init_node('distance_node', anonymous=True)

    # current car's lat and long
    rospy.Subscriber("/reference/rover/piksi/position_receiver_0/sbp/pos_llh", NavSatFix, gps_position_cb, queue_size=1)

    distance_pub = rospy.Publisher('distance', Float64, queue_size=1)

    # -- Waypoints File Finder --
    lane = rospy.get_param("~lane_name") 
    yaml_file_path = f"/waypoints/{lane}.yaml"

    # load waypoints from the YAML file
    with open(yaml_file_path, 'r') as file:
        config = yaml.safe_load(file)

    # intersection and waypoint positions from YAML
    intersections = config['intersections']
    waypoints = config['waypoints']
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
