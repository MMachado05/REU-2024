#!/usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from vehicle_controllers_pkg.cfg import ULCNoYNoRNoGConfig


class SimpleULCVC:
    """
    A simple vehicle control node. It's only capability is to follow a lane line with no
    other understanding of its environment.

    Exists largely for testing purposes.
    """

    lane_twist_subscriber: rospy.Subscriber
    velocity_publisher: rospy.Publisher
    dyn_rcfg_srv: Server

    drive: bool
    speed: float
    twist_multiplier: float

    is_driving: bool

    velocity_msg: Twist

    def __init__(self):
        """
        Initializes the node.
        """
        # Node architecture
        rospy.init_node("simple_gazelle_vc", anonymous=True)

        self.lane_twist_subscriber = rospy.Subscriber(
            rospy.get_param("~lane_twist_in_topic"), Twist, self._follow_lane
        )
        self.velocity_publisher = rospy.Publisher(
            rospy.get_param("~cmd_vel_out_topic"), Twist, queue_size=1
        )
        self.dyn_rcfg_srv = Server(
            ULCNoYNoRNoGConfig, self._dynamic_reconfig_callback
        )

        self.drive_on = False
        self.speed = 0.0
        self.twist_multiplier = 1.0

        self.is_driving = False

        self.velocity_msg = Twist()

        rospy.spin()

    def _dynamic_reconfig_callback(self, config, _):
        """
        Callback for dynamic reconfigure.
        """
        self.drive_on = config.drive_on
        self.speed = config.speed
        self.twist_multiplier = config.twist_multiplier

        return config

    def _follow_lane(self, incoming_twist: Twist) -> None:
        """
        Publishes velocity based on the received lane-following twist information.
        Uses the angle provided by the x and y angular velocities to determine
        yaw rate.

        Parameters
        ----------
        incoming_twist: Twist
            The received twist message guiding the lane following.
        """
        if not self.drive_on:
            if self.is_driving:
                rospy.loginfo("simple_gazelle_vc:81 - stopping vehicle")
                self.is_driving = False
            self.velocity_msg.linear.x = 0.0
            self.velocity_msg.angular.z = 0.0
            self.velocity_publisher.publish(self.velocity_msg)
            return

        if not self.is_driving:
            rospy.loginfo("simple_gazelle_vc:89 - starting vehicle")
            self.is_driving = True
        self.velocity_msg.linear.x = self.speed
        self.velocity_msg.angular.z = -(
            math.atan2(incoming_twist.angular.x, incoming_twist.angular.y)
            * self.twist_multiplier
        )
        # NOTE: Yes, these are swapped. Visually, here's why:
        #
        # What *we* see
        #    x
        #   ──┐
        #     │
        #     │y    What arctan *wants*
        #     │
        #    θ│
        #    \│     / θ    │y
        #           ───────┘
        #              x

        self.velocity_publisher.publish(self.velocity_msg)


if __name__ == "__main__":
    try:
        SimpleULCVC()
    except rospy.ROSInterruptException:
        pass
