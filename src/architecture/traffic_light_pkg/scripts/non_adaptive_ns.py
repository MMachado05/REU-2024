#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Bool, Duration
from traffic_light_pkg.cfg import NonAdaptiveNSConfig

MANUAL = 0
CROSSWALK = 1
INTERSECTION = 2

GREEN = True
RED = False

ZERO_DURATION = rospy.Duration(0)


class NonAdaptiveNorthSouthLight:
    """
    A reconfigurable traffic light node. Does not publish image topics, nor does
    it have adaptive timing capabilities.

    Only expects directions in the four primary cardinal directions.
    """

    # ---------------------------------------
    # --------- Instance Attributes ---------
    # ---------------------------------------
    north_state_pub: rospy.Publisher
    south_state_pub: rospy.Publisher

    north_time_to_next_state_pub: rospy.Publisher
    south_time_to_next_state_pub: rospy.Publisher
    dyn_rcfg_srv: Server

    mode: int

    north_light_state: bool
    south_light_state: bool
    north_light_current_state: bool
    south_light_current_state: bool

    crosswalk_state: bool

    green_duration: rospy.Duration
    red_duration: rospy.Duration

    time_to_next_north_state: rospy.Duration
    time_to_next_south_state: rospy.Duration
    time_of_last_state_change: rospy.Time

    # TODO: THIS DOESN'T WORK!!! Both rospy.spin and while not is shutdown essentially
    #   stop code from running in its tracks, and I can't figure out how to do multi-
    #   threading to fix it. My idea is to have this run everything *exclusively* with
    #   a callback function that will subscribe to a node that's *constantly* publishing
    #   time, so that it *functions* like a "while rospy shutdown blah blah", but being
    #   able to separate them into different nodes so that I can have my cake and eat it,
    #   too.

    # ---------------------------------------
    # --------- Node State Methods ----------
    # ---------------------------------------
    def __init__(self):
        rospy.init_node("non_adaptive_ns_light", anonymous=True)

        self.north_state_pub = rospy.Publisher("north/state", Bool, queue_size=1)
        self.south_state_pub = rospy.Publisher("south/state", Bool, queue_size=1)

        self.north_time_to_next_state_pub = rospy.Publisher(
            "north/time_to_next_state", Duration, queue_size=1
        )
        self.south_time_to_next_state_pub = rospy.Publisher(
            "south/time_to_next_state", Duration, queue_size=1
        )
        self.dyn_rcfg_srv = Server(NonAdaptiveNSConfig, self._dynamic_reconfig_callback)

        self.mode = CROSSWALK

        self.north_light_state = GREEN
        self.south_light_state = GREEN
        self.north_light_current_state = GREEN
        self.south_light_current_state = GREEN

        self.crosswalk_state = GREEN

        self.green_duration = rospy.Duration(45, 0)
        self.red_duration = rospy.Duration(15, 0)

        self.time_to_next_north_state = rospy.Duration(-1, 0)
        self.time_to_next_south_state = rospy.Duration(-1, 0)
        self.time_of_last_state_change = rospy.Time.now()

    def _dynamic_reconfig_callback(self, config, _):
        self.mode = config.mode
        self.north_light_state = config.north_green_light_on
        self.south_green_light_on = config.south_green_light_on
        self.green_duration = rospy.Duration(config.green_duration, 0)
        self.red_duration = rospy.Duration(config.red_duration, 0)
        return config

    # ----------------------------
    # -------- Node Logic --------
    # ----------------------------
    def run(self):
        """
        Begin running this traffic light node.
        """
        while not rospy.is_shutdown():
            rospy.loginfo(self.mode)
            if self.mode == MANUAL:
                self._publish_manual()
            elif self.mode == CROSSWALK:
                self._publish_crosswalk()
            elif self.mode == INTERSECTION:
                self._publish_intersection()
            else:
                rospy.logfatal("non_adaptive_ns_light - Illegal mode.")
                exit(1)

    def _publish_manual(self):
        if self.north_light_state is GREEN and self.north_light_current_state is RED:
            rospy.loginfo("non_adaptive_ns_light - Switching north light to green.")
            self.north_state_pub.publish(Bool(GREEN))
            self.north_light_current_state = GREEN
        if self.north_light_state is RED and self.north_light_current_state is GREEN:
            rospy.loginfo("non_adaptive_ns_light - Switching north light to red.")
            self.north_state_pub.publish(Bool(RED))
            self.north_light_current_state = RED
        if self.south_light_state is GREEN and self.south_light_current_state is RED:
            rospy.loginfo("non_adaptive_ns_light - Switching south light to green.")
            self.south_state_pub.publish(Bool(GREEN))
            self.south_light_current_state = GREEN
        if self.south_light_state is RED and self.south_light_current_state is GREEN:
            rospy.loginfo("non_adaptive_ns_light - Switching north light to red.")
            self.south_state_pub.publish(Bool(RED))
            self.south_light_current_state = RED

    def _publish_crosswalk(self):
        # North and south are the same, so I picked arbitrarily here.
        if self.time_to_next_north_state <= ZERO_DURATION:
            # Checking <= for edge cases where subtracting time elapsed crosses
            #   zero seconds.
            if self.crosswalk_state == GREEN:
                rospy.loginfo("non_adaptive_ns_light - Switching crosswalk to red.")
                self.crosswalk_state = RED
                self.time_to_next_north_state = self.red_duration
                self.time_to_next_south_state = self.red_duration
                self.north_state_pub.publish(Bool(RED))
                self.south_state_pub.publish(Bool(RED))
            elif self.crosswalk_state == RED:
                rospy.loginfo("non_adaptive_ns_light - Switching crosswalk to green.")
                self.crosswalk_state = GREEN
                self.time_to_next_north_state = self.green_duration
                self.time_to_next_south_state = self.green_duration
                self.north_state_pub.publish(Bool(GREEN))
                self.south_state_pub.publish(Bool(GREEN))
            self.time_of_last_state_change = rospy.Time.now()
        else:
            time_elapsed = rospy.Time.now() - self.time_of_last_state_change
            self.time_to_next_north_state = self.time_to_next_north_state - (
                time_elapsed
            )
            self.time_to_next_south_state = self.time_to_next_south_state - (
                time_elapsed
            )
        self.north_time_to_next_state_pub.publish(self.time_to_next_north_state)
        self.south_time_to_next_state_pub.publish(self.time_to_next_south_state)

    def _publish_intersection(self):
        rospy.loginfo(
            "non_adaptive_ns_light - Intersection mode not implemented. Will try again in 10 seconds."
        )
        rospy.sleep(10)


if __name__ == "__main__":
    try:
        light = NonAdaptiveNorthSouthLight()
        light.run()
    except rospy.ROSInterruptException:
        pass
