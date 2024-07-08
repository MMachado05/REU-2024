#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.server import Server
from traffic_light_pkg.cfg import NonAdaptiveNSConfig
from std_msgs.msg import Int32, Bool

class NonAdaptiveNorthSouthLightController:
    """
    A utility class meant to pass dynamically-reconfigured parameters
    to the traffic light via topics.
    """

    dyn_rcfg_srv: Server

    mode_pub: rospy.Publisher
    north_light_state_pub: rospy.Publisher
    south_light_state_pub: rospy.Publisher
    green_duration_pub: rospy.Publisher
    red_duration_pub: rospy.Publisher

    mode: int
    north_light_state: bool
    south_light_state: bool
    green_duration: int
    red_duration: int

    def __init__(self):
        rospy.init_node("non_adaptive_ns_controller", anonymous=True)

        self.mode_pub = rospy.Publisher(
            rospy.get_param("~mode_topic"), Int32, queue_size=1
        )
        self.north_light_state_pub = rospy.Publisher(
            rospy.get_param("~north_light_state_topic"), Bool, queue_size=1
        )
        self.south_light_state_pub = rospy.Publisher(
            rospy.get_param("~south_light_state_topic"), Bool, queue_size=1
        )
        self.green_duration_pub = rospy.Publisher(
            rospy.get_param("~green_duration_topic"), Int32, queue_size=1
        )
        self.red_duration_pub = rospy.Publisher(
            rospy.get_param("~red_duration_topic"), Int32, queue_size=1
        )

        self.dyn_rcfg_srv = Server(
            NonAdaptiveNSConfig,
            self._push_traffic_light_params
        )

        self.mode = 1
        self.north_light_state = True
        self.south_light_state = True
        self.green_duration = 45
        self.red_duration = 15

        self.mode_pub.publish(self.mode)
        self.north_light_state_pub.publish(self.north_light_state)
        self.south_light_state_pub.publish(self.south_light_state)
        self.green_duration_pub.publish(self.green_duration)
        self.red_duration_pub.publish(self.red_duration)

        rospy.spin()

    def _push_traffic_light_params(self, config, _):
        self.mode = config.mode
        self.north_light_state = config.north_green_light_on
        self.south_light_state = config.south_green_light_on
        self.green_duration = config.green_duration
        self.red_duration = config.red_duration

        self.mode_pub.publish(self.mode)
        self.north_light_state_pub.publish(self.north_light_state)
        self.south_light_state_pub.publish(self.south_light_state)
        self.green_duration_pub.publish(self.green_duration)
        self.red_duration_pub.publish(self.red_duration)
        return config

if __name__ == "__main__":
    try:
        NonAdaptiveNorthSouthLightController()
    except rospy.ROSInterruptException:
        pass
