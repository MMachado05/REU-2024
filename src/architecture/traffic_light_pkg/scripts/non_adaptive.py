#!/usr/bin/env python3

import rospy
from typing import Dict, List
from std_msgs.msg import Bool
from std_msgs.msg import Time


class NonAdaptiveLight:
    """
    A reconfigurable traffic light node. Does not publish image topics, nor does
    it have adaptive timing capabilities.
    """

    directions: List[str]
    direction_to_state_pubs: Dict[str, rospy.Publisher]
    direction_to_time_to_state_pubs: Dict[str, rospy.Publisher]

    def __init__(self):
        rospy.init_node("non_adaptive_light", anonymous=True)

        directions_temp = rospy.get_param("~directions")
        if type(directions_temp) is str:
            self.directions = directions_temp.split()
        else:
            rospy.logfatal(
                "non_adaptive_light - Unable to parse directions from launch file."
            )
            exit(1)

        self.direction_to_state_pubs = {}
        self.direction_to_time_to_state_pubs = {}
        for direction in self.directions:
            self.direction_to_state_pubs[direction] = rospy.Publisher(
                f"/{direction}/state", Bool, queue_size=1
            )
            self.direction_to_time_to_state_pubs[direction] = rospy.Publisher(
                f"/{direction}/time_to_state", Time, queue_size=1
            )

        rospy.spin()
        # TODO: Literally this entire node.
