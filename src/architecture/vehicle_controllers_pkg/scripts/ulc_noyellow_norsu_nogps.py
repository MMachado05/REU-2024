#!/usr/bin/env python3

import rospy
import math

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from vehicle_controllers_pkg.cfg import ULCNoYNoRNoGConfig
from dataspeed_ulc_msgs.msg import UlcCmd  # Drive by wire UL
from dbw_polaris_msgs.msg import SteeringCmd  # drive by wire native messages


class SimpleULCVC:
    """
    A simple vehicle control node. It's only capability is to follow a lane line with no
    other understanding of its environment.

    Exists largely for testing purposes.
    """

    lane_twist_subscriber: rospy.Subscriber
    dyn_rcfg_srv: Server

    drive: bool
    speed: float
    twist_multiplier: float

    is_driving: bool

    def __init__(self):
        """
        Initializes the node.
        """
        # Node architecture
        rospy.init_node("simple_ulc_vc", anonymous=True)

        self.lane_twist_subscriber = rospy.Subscriber(
            rospy.get_param("~lane_twist_in_topic"), Twist, self._follow_lane
        )
        self.dyn_rcfg_srv = Server(ULCNoYNoRNoGConfig, self._dynamic_reconfig_callback)

        # ULC-related node architecture
        self.ulc_speed_publisher = rospy.Publisher(
            "vehicle/ulc_cmd", UlcCmd, queue_size=1
        )
        self.ulc_steering_publisher = rospy.Publisher(
            "vehicle/steering_cmd", SteeringCmd, queue_size=1
        )
        self.ulc_enable_publisher = rospy.Publisher(
            "vehicle/enable", Empty, queue_size=1
        )

        self.drive_on = False
        self.speed = 0.0
        self.twist_multiplier = 1.0

        self.is_driving = False

        # ----- Prepare speed message -----------
        self.speed_msg = UlcCmd()
        self.speed_msg.enable_pedals = True
        self.speed_msg.enable_steering = False  # Steering control via ULC not used here
        self.speed_msg.enable_shifting = True
        self.speed_msg.shift_from_park = True
        self.speed_msg.linear_accel = 0.0
        self.speed_msg.linear_decel = 0.0
        self.speed_msg.jerk_limit_throttle = 0.0
        self.speed_msg.jerk_limit_brake = 0.0
        self.speed_msg.pedals_mode = 0  # Speed mode
        # ---------------------------------------

        # ------ Prepare steer message ----------
        self.steer_msg = SteeringCmd()
        self.steer_msg.enable = (
            True  # Enable Steering, required 'True' for control via ROS
        )

        # Do NOT use these without completely understanding how they work on the hardware level:
        self.steer_msg.cmd_type = SteeringCmd.CMD_ANGLE
        # CAUTION: Torque mode disables lateral acceleration limits
        # Use angle velocity to control rate. Lock to lock = 1200deg i.e. 300deg/s will be 4secs lock to lock
        self.steer_msg.steering_wheel_angle_velocity = math.radians(300)
        # deg/s -> rad/s

        self.steer_msg.steering_wheel_torque_cmd = 0.0  # Nm
        self.steer_msg.clear = False
        self.steer_msg.ignore = False
        self.steer_msg.calibrate = False
        self.steer_msg.quiet = False
        self.steer_msg.count = 0
        # ---------------------------------------

        # Prepare message for enabling DBW
        self.empty_msg = Empty()

    def _dynamic_reconfig_callback(self, config, _):
        """
        Callback for dynamic reconfigure.
        """
        self.drive_on = config.drive_on
        self.speed = config.speed
        self.twist_multiplier = config.twist_multiplier

        return config

    # ----------------------------
    # --- Lane following logic ---
    # ----------------------------
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

            self._prep_steering_angle(0)
            self.speed_msg.linear_velocity = 0

            self.ulc_speed_publisher.publish(self.speed_msg)
            self.ulc_steering_publisher.publish(self.steer_msg)
            return

        self.ulc_enable_publisher.publish(self.empty_msg)
        if not self.is_driving:
            rospy.loginfo("simple_gazelle_vc:89 - starting vehicle")
            self.is_driving = True
        turn_angle = -(
            math.atan2(incoming_twist.angular.x, incoming_twist.angular.y)
            * self.twist_multiplier
        )
        # NOTE: Yes, these are swapped. Visually, here's why:
        # ..
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

        self.speed_msg.linear_velocity = self.speed
        self._prep_steering_angle(turn_angle)

        self.ulc_speed_publisher.publish(self.speed_msg)
        self.ulc_steering_publisher.publish(self.steer_msg)

    # ---------------------------------------
    # --- ULC Publishing helper functions ---
    # ---------------------------------------
    def _prep_steering_angle(self, requested_road_angle: float) -> None:
        """
        Prepares self.steer_msg to be published
        """
        if requested_road_angle < 0:
            requested_road_angle = max(requested_road_angle, -37) * 16.2
        else:
            requested_road_angle = min(requested_road_angle, 37) * 16.2
        self.steer_msg.steering_wheel_angle_cmd = math.radians(requested_road_angle)
        # (-600deg to 600deg converted to radians)


if __name__ == "__main__":
    try:
        vc = SimpleULCVC()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
