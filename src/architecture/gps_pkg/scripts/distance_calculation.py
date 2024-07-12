#!/usr/bin/env python3

import rospkg
import rospy
import yaml
from geopy.distance import geodesic
from libsbp_ros_msgs.msg import MsgPosLlh
from std_msgs.msg import Float64

distance = -1

def gps_position_cb(msg):
    global gps_lat, gps_lon, current_pose, distance

    gps_lat = msg.lat
    gps_lon = msg.lon

    rospy.loginfo(f'lat: {gps_lat} long: {gps_lon}')

    current_pose = {'lat': gps_lat, 'lon': gps_lon}

    distance = calculate_distance()
    distance_pub.publish(distance)

def calculate_distance():
    global intersections, waypoints, total_distance

    # find the closest waypoint to the current position
    closest_idx = -1
    min_distance = float('inf')
    for i, waypoint in enumerate(waypoints):
        dist = geodesic((current_pose['lat'], current_pose['lon']), (waypoint[0], waypoint[1])).meters

        if dist < min_distance:
            min_distance = dist
            closest_idx = i

    # sum the total distance to the next intersection
    total_distance = min_distance
    for i in range(closest_idx, len(waypoints) - 1):
        total_distance += geodesic((waypoints[i][0], waypoints[i][1]), (waypoints[i + 1][0], waypoints[i + 1][1])).meters

    return total_distance

def open_waypoints(name):
    global intersections, waypoints

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('gps_pkg')
    yaml_file_path = f"{package_path}/waypoints/{name}.yaml"

    # load waypoints from the YAML file
    with open(yaml_file_path, 'r') as file:
        config = yaml.safe_load(file)

    # intersection and waypoint positions from YAML
    intersections = config['intersections']
    waypoints = config['waypoints']

    return

if __name__ == '__main__':
    rospy.init_node('distance_calculation')
    rospy.loginfo('distance_calculation started ... ')

    lane_name = rospy.get_param('~lane_name')
    
    open_waypoints(lane_name)

    distance_pub = rospy.Publisher('distance', Float64, queue_size=1)

    gps_sub = rospy.Subscriber("reference/piksi/position_receiver_0/sbp/pos_llh", MsgPosLlh, gps_position_cb, queue_size=1)

    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
