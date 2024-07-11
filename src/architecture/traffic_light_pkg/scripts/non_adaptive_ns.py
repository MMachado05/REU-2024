#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Time, Int32
from typing import Optional

MANUAL = 0
CROSSWALK = 1
INTERSECTION = 2

MODE_INT_TO_STR = {
    MANUAL: "MANUAL",
    CROSSWALK: "CROSSWALK",
    INTERSECTION: "INTERSECTION",
}

GREEN = True
RED = False

NORTHBOUND = 3
EASTBOUND = 4
ALL_RED = 5

STATE_BOOL_TO_STR = {GREEN: "GREEN", RED: "RED"}

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
    run_sub: rospy.Subscriber
    mode_sub: rospy.Subscriber
    northbound_light_state_sub: rospy.Subscriber
    eastbound_light_state_sub: rospy.Subscriber
    green_duration_sub: rospy.Subscriber
    red_duration_sub: rospy.Subscriber

    northbound_state_pub: rospy.Publisher
    eastbound_state_pub: rospy.Publisher

    northbound_time_to_next_state_pub: rospy.Publisher
    eastbound_time_to_next_state_pub: rospy.Publisher

    mode: int

    northbound_light_state: bool
    eastbound_light_state: bool
    northbound_light_current_state: bool
    eastbound_light_current_state: bool

    crosswalk_state: bool
    intersection_state: int

    green_duration: rospy.Duration
    red_duration: rospy.Duration

    time_to_next_northbound_state: rospy.Duration
    time_to_next_eastbound_state: rospy.Duration
    time_of_last_state_change: Optional[rospy.Time]

    # ---------------------------------------
    # --------- Node State Methods ----------
    # ---------------------------------------
    def __init__(self):
        rospy.init_node("non_adaptive_ns_light", anonymous=True)

        self.mode = CROSSWALK
        self.northbound_light_state = GREEN
        self.eastbound_light_state = GREEN
        self.green_duration = rospy.Duration(45, 0)
        self.red_duration = rospy.Duration(15, 0)

        self.northbound_light_current_state = GREEN
        self.eastbound_light_current_state = GREEN

        self.crosswalk_state = GREEN
        self.intersection_state = ALL_RED

        self.time_to_next_northbound_state = rospy.Duration(-1, 0)
        self.time_to_next_eastbound_state = rospy.Duration(-1, 0)
        self.time_of_last_state_change = None

        self.run_sub = rospy.Subscriber(
            rospy.get_param("~run_light_topic"), Time, self._run
        )

        self.mode_sub = rospy.Subscriber(
            rospy.get_param("~mode_topic"), Int32, self._set_mode
        )
        self.northbound_light_state_sub = rospy.Subscriber(
            rospy.get_param("~northbound_light_state_topic"),
            Bool,
            self._set_northbound_light_state,
        )
        self.eastbound_light_state_sub = rospy.Subscriber(
            rospy.get_param("~eastbound_light_state_topic"),
            Bool,
            self._set_eastbound_light_state,
        )
        self.green_duration_sub = rospy.Subscriber(
            rospy.get_param("~green_duration_topic"), Int32, self._set_green_duration
        )
        self.red_duration_sub = rospy.Subscriber(
            rospy.get_param("~red_duration_topic"), Int32, self._set_red_duration
        )

        self.northbound_state_pub = rospy.Publisher(
            "northbound/state", Bool, queue_size=1
        )
        self.eastbound_state_pub = rospy.Publisher(
            "eastbound/state", Bool, queue_size=1
        )

        self.northbound_time_to_next_state_pub = rospy.Publisher(
            "northbound/time_to_next_state", Time, queue_size=1
        )
        self.eastbound_time_to_next_state_pub = rospy.Publisher(
            "eastbound/time_to_next_state", Time, queue_size=1
        )

    # -------------------------------------------------------
    # ------ Setters from external dynamic reconfigure ------
    # -------------------------------------------------------
    # This is so we can use dynamic reconfigure for a node running on the desktop-less
    #   RSU pi.
    def _set_mode(self, mode: Int32):
        if self.mode != mode.data:
            rospy.loginfo(
                "non_adaptive_ns_light - Mode changed from {} to {}".format(
                    MODE_INT_TO_STR[self.mode], MODE_INT_TO_STR[mode.data]
                )
            )

            rospy.loginfo(
                "non_adaptive_ns_light - North light is {}, eastbound light is {}".format(
                    STATE_BOOL_TO_STR[self.northbound_light_state],
                    STATE_BOOL_TO_STR[self.eastbound_light_state],
                )
            )
            self.northbound_state_pub.publish(self.northbound_light_state)
            self.eastbound_state_pub.publish(self.eastbound_light_state)
        self.mode = mode.data

    def _set_northbound_light_state(self, state: Bool):
        self.northbound_light_state = state.data

    def _set_eastbound_light_state(self, state: Bool):
        self.eastbound_light_state = state.data

    def _set_green_duration(self, duration: Int32):
        self.green_duration = rospy.Duration(duration.data, 0)

    def _set_red_duration(self, duration: Int32):
        self.red_duration = rospy.Duration(duration.data, 0)

    # ----------------------------
    # -------- Node Logic --------
    # ----------------------------
    def _run(self, now: Time):
        if self.mode == MANUAL:
            self._publish_manual()
            self.northbound_time_to_next_state_pub.publish(rospy.Duration(-10))
            self.eastbound_time_to_next_state_pub.publish(rospy.Duration(-10))
        elif self.mode == CROSSWALK:
            self._publish_crosswalk(rospy.Time(now.data.secs, now.data.nsecs))
        elif self.mode == INTERSECTION:
            self._publish_intersection(rospy.Time(now.data.secs, now.data.nsecs))
        else:
            rospy.logfatal(f"non_adaptive_ns_light - Illegal mode <{self.mode}>.")
            exit(1)

    def _publish_manual(self):
        if (
            self.northbound_light_state is GREEN
            and self.northbound_light_current_state is RED
        ):
            rospy.loginfo(
                "non_adaptive_ns_light - Switching northbound light to green."
            )
            self.northbound_state_pub.publish(Bool(GREEN))
            self.northbound_light_current_state = GREEN
        if (
            self.northbound_light_state is RED
            and self.northbound_light_current_state is GREEN
        ):
            rospy.loginfo("non_adaptive_ns_light - Switching northbound light to red.")
            self.northbound_state_pub.publish(Bool(RED))
            self.northbound_light_current_state = RED
        if (
            self.eastbound_light_state is GREEN
            and self.eastbound_light_current_state is RED
        ):
            rospy.loginfo("non_adaptive_ns_light - Switching eastbound light to green.")
            self.eastbound_state_pub.publish(Bool(GREEN))
            self.eastbound_light_current_state = GREEN
        if (
            self.eastbound_light_state is RED
            and self.eastbound_light_current_state is GREEN
        ):
            rospy.loginfo("non_adaptive_ns_light - Switching eastbound light to red.")
            self.eastbound_state_pub.publish(Bool(RED))
            self.eastbound_light_current_state = RED

    def _publish_crosswalk(self, now: rospy.Time):
        if self.time_of_last_state_change is None:
            self.time_of_last_state_change = rospy.Time(now.secs, now.nsecs)

        # Get time elapsed since last state change.
        time_elapsed_s = now.secs - self.time_of_last_state_change.secs
        time_elapsed_ns = now.nsecs - self.time_of_last_state_change.nsecs
        time_since_last_state = rospy.Duration(time_elapsed_s, time_elapsed_ns)

        # Change states if necessary
        if (
            self.crosswalk_state == GREEN
            and time_since_last_state >= self.green_duration
        ):
            rospy.loginfo("non_adaptive_ns_light - Switching crosswalk to red.")
            self.crosswalk_state = RED
            self.time_to_next_northbound_state = self.red_duration
            self.time_to_next_eastbound_state = self.red_duration
            self.northbound_state_pub.publish(Bool(RED))
            self.eastbound_state_pub.publish(Bool(RED))
            self.time_of_last_state_change = now
        elif self.crosswalk_state == RED and time_since_last_state >= self.red_duration:
            rospy.loginfo("non_adaptive_ns_light - Switching crosswalk to green.")
            self.crosswalk_state = GREEN
            self.time_to_next_northbound_state = self.green_duration
            self.time_to_next_eastbound_state = self.green_duration
            self.northbound_state_pub.publish(Bool(GREEN))
            self.eastbound_state_pub.publish(Bool(GREEN))
            self.time_of_last_state_change = now
        elif self.crosswalk_state == GREEN:
            self.time_to_next_northbound_state = (
                self.green_duration - time_since_last_state
            )
            self.time_to_next_eastbound_state = (
                self.green_duration - time_since_last_state
            )
        elif self.crosswalk_state == RED:
            self.time_to_next_northbound_state = (
                self.red_duration - time_since_last_state
            )
            self.time_to_next_eastbound_state = (
                self.red_duration - time_since_last_state
            )
        else:
            rospy.logerr(
                "non_adaptive_ns_light:257 - Unfamiliar crosswalk and duration configuration."
            )
            return

        self.northbound_time_to_next_state_pub.publish(
            self.time_to_next_northbound_state
        )
        self.eastbound_time_to_next_state_pub.publish(self.time_to_next_eastbound_state)

    def _publish_intersection(self, now: rospy.Time):
        if self.time_of_last_state_change is None:
            self.time_of_last_state_change = rospy.Time(now.secs, now.nsecs)
        rospy.loginfo(
            "non_adaptive_ns_light - Intersection mode not implemented. Will try again in 10 seconds."
        )
        rospy.sleep(10)


if __name__ == "__main__":
    try:
        light = NonAdaptiveNorthSouthLight()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
