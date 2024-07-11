#!/usr/bin/env python3

import rospy
import math

from std_msgs.msg import Empty, Float64, Int32, Bool, Time
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from vehicle_controllers_pkg.cfg import ULCNoYNoRNoGConfig
from dataspeed_ulc_msgs.msg import UlcCmd  # Drive by wire UL
from dbw_polaris_msgs.msg import SteeringCmd  # drive by wire native messages

LENGTH_OF_CRIT_ZONE = 5.0
TIME_TOLERANCE = 1.5

RED = False
GREEN = True


class ULCWithV2XNoYellowVC:
    # TODO: Subscribe to mode and change algorithm based on mode
    """
    A simple vehicle control node. It's only capability is to follow a lane line with no
    other understanding of its environment.

    Exists largely for testing purposes.
    """

    lane_twist_subscriber: rospy.Subscriber
    distance_subscriber: rospy.Subscriber
    green_duration_subscriber: rospy.Subscriber
    red_duration_subscriber: rospy.Subscriber
    light_state_subscriber: rospy.Subscriber
    light_time_to_next_state_subscriber: rospy.Subscriber
    dyn_rcfg_srv: Server

    drive: bool
    speed: float
    twist_multiplier: float

    is_driving: bool

    # Light stuff
    distance_from_intersection: float

    current_light_state: bool
    time_to_next_state: rospy.Duration

    red_duration: int
    green_duration: int

    def __init__(self):
        """
        Initializes the node.
        """
        # Node architecture
        rospy.init_node("simple_ulc_vc", anonymous=True)
        
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

        lane_name = rospy.get_param("~lane_name")

        self.light_state_subscriber = rospy.Subscriber(
            f"/light/{lane_name}/state", Bool, self._get_light_state
        )
        self.light_time_to_next_state_subscriber = rospy.Subscriber(
            f"/light/{lane_name}/time_to_next_state",
            Bool,
            self._get_light_time_to_next_state,
        )

        self.green_duration_subscriber = rospy.Subscriber(
            "/light/green_duration", Int32, self._get_green_duration
        )
        self.red_duration_subscriber = rospy.Subscriber(
            "/light/red_duration", Int32, self._get_red_duration
        )

        self.lane_twist_subscriber = rospy.Subscriber(
            rospy.get_param("~lane_twist_in_topic"), Twist, self._follow_lane
        )
        self.distance_subscriber = rospy.Subscriber(
            rospy.get_param("~distance_from_intersection_topic"),
            Float64,
            self._get_distance,
        )
        self.dyn_rcfg_srv = Server(ULCNoYNoRNoGConfig, self._dynamic_reconfig_callback)

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

        # Misc.
        self.distance_from_intersection

    def _dynamic_reconfig_callback(self, config, _):
        """
        Callback for dynamic reconfigure.
        """
        self.drive_on = config.drive_on
        self.speed = config.speed
        self.twist_multiplier = config.twist_multiplier

        return config

    def _get_distance(self, distance: Float64):
        self.distance_from_intersection = distance.data

    def _get_red_duration(self, red_duration: Int32):
        self.red_duration = red_duration.data

    def _get_green_duration(self, green_duration: Int32):
        self.green_duration = green_duration.data

    def _get_light_state(self, state: Bool):
        self.current_light_state = state.data

    def _get_light_time_to_next_state(self, time: Time):
        self.time_to_next_state = rospy.Duration(time.data.secs, time.data.nsecs)

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
        turn_angle = math.degrees(turn_angle)
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
        distance_to_end_of_intersection = (
            self.distance_from_intersection + LENGTH_OF_CRIT_ZONE
        )
        final_speed = self.speed
        time_left_as_float = float(
            str(self.time_to_next_state.secs) + "." + str(self.time_to_next_state.nsecs)
        )

        potential_distance = self.speed * self.time_to_next_state
        if self.current_light_state == RED:
            if potential_distance > self.distance_from_intersection:
                final_speed = self.distance_from_intersection / (
                    time_left_as_float + TIME_TOLERANCE
                )
        else:
            if potential_distance < self.distance_from_intersection:
                potential_distance = self.speed * (
                    time_left_as_float + self.red_duration + TIME_TOLERANCE
                )
                if potential_distance < self.distance_from_intersection:
                    final_speed = self.distance_from_intersection / (
                        time_left_as_float + self.red_duration + TIME_TOLERANCE
                    )

        self.speed_msg.linear_velocity = final_speed
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
        vc = ULCWithV2XNoYellowVC()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
