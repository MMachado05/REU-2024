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

NORTHBOUND_GREEN = 3
ALL_RED_NEXT_EAST = 4
EASTBOUND_GREEN = 5
ALL_RED_NEXT_NORTH = 6

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

    northbound_duration_of_next_state_pub: rospy.Publisher
    eastbound_duration_of_next_state_pub: rospy.Publisher

    crosswalk_state: bool
    intersection_state: int

    green_duration: int
    red_duration: int
    intersection_total_red_duration: int

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
        self.green_duration = 45
        self.red_duration = 15
        self.intersection_total_red_duration = 75

        self.northbound_light_current_state = GREEN
        self.eastbound_light_current_state = GREEN

        self.crosswalk_state = GREEN
        self.intersection_state = ALL_RED_NEXT_EAST

        self.time_to_next_northbound_state = rospy.Duration(-1, 0)
        self.time_to_next_eastbound_state = rospy.Duration(-1, 0)
        self.time_of_last_state_change = None

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

        self.northbound_duration_of_next_state_pub = rospy.Publisher(
            "northbound/duration_of_next_state", Int32, queue_size=1
        )
        self.eastbound_duration_of_next_state_pub = rospy.Publisher(
            "eastbound/duration_of_next_state", Int32, queue_size=1
        )

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
        self.green_duration = duration.data
        self.intersection_total_red_duration = (
            self.red_duration + self.green_duration + self.red_duration
        )

        # Publish next state duration, if next state is green
        if self.mode == CROSSWALK:
            if self.crosswalk_state == RED:
                self.northbound_duration_of_next_state_pub.publish(self.green_duration)
                self.eastbound_duration_of_next_state_pub.publish(self.green_duration)
        elif self.mode == INTERSECTION:
            if self.intersection_state == EASTBOUND_GREEN:
                self.eastbound_duration_of_next_state_pub.publish(
                    self.red_duration + self.green_duration + self.red_duration
                )
                self.northbound_duration_of_next_state_pub.publish(self.green_duration)
            elif self.intersection_state == NORTHBOUND_GREEN:
                self.northbound_duration_of_next_state_pub.publish(
                    self.red_duration + self.green_duration + self.red_duration
                )
                self.eastbound_duration_of_next_state_pub.publish(self.green_duration)
            else:  # All red, either kind
                self.northbound_duration_of_next_state_pub.publish(self.green_duration)
                self.eastbound_duration_of_next_state_pub.publish(self.green_duration)

    def _set_red_duration(self, duration: Int32):
        self.red_duration = duration.data
        self.intersection_total_red_duration = (
            self.red_duration + self.green_duration + self.red_duration
        )

        # Publish next state duration, if next state is green
        if self.mode == CROSSWALK:
            if self.crosswalk_state == GREEN:
                self.northbound_duration_of_next_state_pub.publish(self.red_duration)
                self.eastbound_duration_of_next_state_pub.publish(self.red_duration)
        elif self.mode == INTERSECTION:
            if self.intersection_state == EASTBOUND_GREEN:
                self.eastbound_duration_of_next_state_pub.publish(
                    self.green_duration + self.red_duration
                )
            elif self.intersection_state == NORTHBOUND_GREEN:
                self.northbound_duration_of_next_state_pub.publish(
                    self.green_duration + self.red_duration
                )

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

        # Create rospy.Durations for duration instance attributes
        green_duration_time = rospy.Duration(self.green_duration)
        red_duration_time = rospy.Duration(self.red_duration)

        # Change states if necessary
        if (
            self.crosswalk_state == GREEN
            and time_since_last_state >= green_duration_time
        ):
            rospy.loginfo("non_adaptive_ns_light - Crosswalk is now RED.")

            self.crosswalk_state = RED
            self.time_to_next_northbound_state = red_duration_time
            self.time_to_next_eastbound_state = red_duration_time

            self.northbound_state_pub.publish(Bool(RED))
            self.eastbound_state_pub.publish(Bool(RED))

            self.northbound_duration_of_next_state_pub.publish(self.green_duration)
            self.eastbound_duration_of_next_state_pub.publish(self.green_duration)

            self.time_of_last_state_change = now
        elif self.crosswalk_state == RED and time_since_last_state >= red_duration_time:
            rospy.loginfo("non_adaptive_ns_light - Crosswalk is now GREEN.")

            self.crosswalk_state = GREEN
            self.time_to_next_northbound_state = green_duration_time
            self.time_to_next_eastbound_state = green_duration_time

            self.northbound_state_pub.publish(Bool(GREEN))
            self.eastbound_state_pub.publish(Bool(GREEN))

            self.northbound_duration_of_next_state_pub.publish(self.red_duration)
            self.eastbound_duration_of_next_state_pub.publish(self.red_duration)

            self.time_of_last_state_change = now
        elif self.crosswalk_state == GREEN:
            self.time_to_next_northbound_state = (
                green_duration_time - time_since_last_state
            )
            self.time_to_next_eastbound_state = (
                green_duration_time - time_since_last_state
            )
        elif self.crosswalk_state == RED:
            self.time_to_next_northbound_state = (
                red_duration_time - time_since_last_state
            )
            self.time_to_next_eastbound_state = (
                red_duration_time - time_since_last_state
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

        # Get time elapsed since last state change.
        time_elapsed_s = now.secs - self.time_of_last_state_change.secs
        time_elapsed_ns = now.nsecs - self.time_of_last_state_change.nsecs
        time_since_last_state = rospy.Duration(time_elapsed_s, time_elapsed_ns)

        # Create rospy.Durations for duration instance attributes
        green_duration_time = rospy.Duration(self.green_duration)
        red_duration_time = rospy.Duration(self.red_duration)
        total_red_time = rospy.Duration(self.intersection_total_red_duration)

        # State changes
        if (
            self.intersection_state == NORTHBOUND_GREEN
            and time_since_last_state >= green_duration_time
        ):
            rospy.loginfo(
                "non_adaptive_ns_light - ALL RED, EAST will be GREEN."
            )
            self.intersection_state = ALL_RED_NEXT_EAST

            self.time_to_next_northbound_state = total_red_time
            self.northbound_state_pub.publish(Bool(RED))
            self.northbound_duration_of_next_state_pub.publish(self.green_duration)

            self.time_of_last_state_change = now
        elif (
            self.intersection_state == ALL_RED_NEXT_EAST
            and time_since_last_state >= red_duration_time
        ):
            rospy.loginfo("non_adaptive_ns_light - EAST light is GREEN.")
            self.intersection_state = EASTBOUND_GREEN

            self.time_to_next_eastbound_state = green_duration_time

            self.eastbound_state_pub.publish(Bool(GREEN))
            self.eastbound_duration_of_next_state_pub.publish(
                self.intersection_total_red_duration
            )

            self.time_of_last_state_change = now
        elif (
            self.intersection_state == EASTBOUND_GREEN
            and time_since_last_state >= green_duration_time
        ):
            rospy.loginfo(
                "non_adaptive_ns_light - ALL RED, NORTH will be GREEN."
            )
            self.intersection_state = ALL_RED_NEXT_NORTH

            self.time_to_next_eastbound_state = total_red_time
            self.eastbound_state_pub.publish(Bool(RED))
            self.eastbound_duration_of_next_state_pub.publish(self.green_duration)

            self.time_of_last_state_change = now
        elif (
            self.intersection_state == ALL_RED_NEXT_NORTH
            and time_since_last_state >= red_duration_time
        ):
            rospy.loginfo("non_adaptive_ns_light - NORTH light is GREEN.")
            self.intersection_state = NORTHBOUND_GREEN

            self.time_to_next_northbound_state = green_duration_time

            self.northbound_state_pub.publish(Bool(GREEN))
            self.northbound_duration_of_next_state_pub.publish(
                self.intersection_total_red_duration
            )

            self.time_of_last_state_change = now

        # Non-state changes
        elif self.intersection_state == NORTHBOUND_GREEN:
            self.time_to_next_northbound_state = (
                green_duration_time - time_since_last_state
            )
            self.time_to_next_eastbound_state = (
                red_duration_time + green_duration_time
            ) - time_since_last_state
        elif self.intersection_state == ALL_RED_NEXT_EAST:
            self.time_to_next_northbound_state = (
                total_red_time - time_since_last_state
            )
            self.time_to_next_eastbound_state = (
                red_duration_time - time_since_last_state
            )
        elif self.intersection_state == EASTBOUND_GREEN:
            self.time_to_next_eastbound_state = (
                green_duration_time - time_since_last_state
            )
            self.time_to_next_northbound_state = (
                red_duration_time + green_duration_time
            ) - time_since_last_state
        elif self.intersection_state == ALL_RED_NEXT_NORTH:
            self.time_to_next_eastbound_state = (
                total_red_time - time_since_last_state
            )
            self.time_to_next_northbound_state = (
                red_duration_time - time_since_last_state
            )

        else:
            rospy.logerr(
                "non_adaptive_ns_light:339 - Unfamiliar intersection and duration configuration."
            )
            return

        self.northbound_time_to_next_state_pub.publish(
            self.time_to_next_northbound_state
        )
        self.eastbound_time_to_next_state_pub.publish(self.time_to_next_eastbound_state)


if __name__ == "__main__":
    try:
        light = NonAdaptiveNorthSouthLight()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
