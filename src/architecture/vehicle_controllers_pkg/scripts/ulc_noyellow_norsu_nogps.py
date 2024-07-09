#!/usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from vehicle_controllers_pkg.cfg import ULCNoYNoRNoGConfig
from dataspeed_ulc_msgs.msg import UlcCmd # Drive by wire UL 
from dbw_polaris_msgs.msg import SteeringCmd # drive by wire native messages


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
            publish_ulc_speed(0)
            publish_steering(0)
            self.velocity_msg.angular.z = 0.0
            self.velocity_publisher.publish(self.velocity_msg)
            return

        if not self.is_driving:
            rospy.loginfo("simple_gazelle_vc:89 - starting vehicle")
            self.is_driving = True
        publish_ulc_speed(self.speed)
        turn_angle = -(
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

        publish_steering(turn_angle)
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


if __name__ == "__main__":
    try:
        SimpleULCVC()
    except rospy.ROSInterruptException:
        pass
