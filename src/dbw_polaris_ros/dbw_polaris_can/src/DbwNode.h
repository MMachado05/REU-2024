/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef _DBW_NODE_H_
#define _DBW_NODE_H_

#include <ros/ros.h>

// ROS messages
#include <can_msgs/Frame.h>
#include <dataspeed_can_msg_filters/ApproximateTime.h>
#include <dbw_polaris_msgs/BrakeCmd.h>
#include <dbw_polaris_msgs/BrakeReport.h>
#include <dbw_polaris_msgs/ThrottleCmd.h>
#include <dbw_polaris_msgs/ThrottleReport.h>
#include <dbw_polaris_msgs/SteeringCmd.h>
#include <dbw_polaris_msgs/SteeringReport.h>
#include <dbw_polaris_msgs/GearCmd.h>
#include <dbw_polaris_msgs/GearReport.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// Platform and module version map
#include <dbw_polaris_can/PlatformMap.h>

namespace dbw_polaris_can
{

class DbwNode
{
public:
  DbwNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
  ~DbwNode();

private:
  void timerCallback(const ros::TimerEvent& event);
  void recvEnable(const std_msgs::Empty::ConstPtr& msg);
  void recvDisable(const std_msgs::Empty::ConstPtr& msg);
  void recvCAN(const can_msgs::Frame::ConstPtr& msg);
  void recvCanImu(const std::vector<can_msgs::Frame::ConstPtr> &msgs);
  void recvBrakeCmd(const dbw_polaris_msgs::BrakeCmd::ConstPtr& msg);
  void recvThrottleCmd(const dbw_polaris_msgs::ThrottleCmd::ConstPtr& msg);
  void recvSteeringCmd(const dbw_polaris_msgs::SteeringCmd::ConstPtr& msg);
  void recvGearCmd(const dbw_polaris_msgs::GearCmd::ConstPtr& msg);
  void recvCalibrateSteering(const std_msgs::Empty::ConstPtr& msg);

  ros::Timer timer_;
  bool prev_enable_;
  bool enable_;
  bool override_brake_;
  bool override_throttle_;
  bool override_steering_;
  bool override_gear_;
  bool fault_brakes_;
  bool fault_throttle_;
  bool fault_steering_;
  bool fault_steering_cal_;
  bool fault_watchdog_;
  bool fault_watchdog_using_brakes_;
  bool fault_watchdog_warned_;
  bool timeout_brakes_;
  bool timeout_throttle_;
  bool timeout_steering_;
  bool enabled_brakes_;
  bool enabled_throttle_;
  bool enabled_steering_;
  bool gear_warned_;
  inline bool fault() { return fault_brakes_ || fault_throttle_ || fault_steering_ || fault_steering_cal_ || fault_watchdog_; }
  inline bool override() { return override_brake_ || override_throttle_ || override_steering_ || override_gear_; }
  inline bool clear() { return enable_ && override(); }
  inline bool enabled() { return enable_ && !fault() && !override(); }
  bool publishDbwEnabled(bool force = false);
  void enableSystem();
  void disableSystem();
  void buttonCancel();
  void overrideBrake(bool override, bool timeout);
  void overrideThrottle(bool override, bool timeout);
  void overrideSteering(bool override, bool timeout);
  void overrideGear(bool override);
  void timeoutBrake(bool timeout, bool enabled);
  void timeoutThrottle(bool timeout, bool enabled);
  void timeoutSteering(bool timeout, bool enabled);
  void faultBrakes(bool fault);
  void faultThrottle(bool fault);
  void faultSteering(bool fault);
  void faultSteeringCal(bool fault);
  void faultWatchdog(bool fault, uint8_t src, bool braking);
  void faultWatchdog(bool fault, uint8_t src = 0);

  enum {
    JOINT_FL = 0, // Front left wheel
    JOINT_FR, // Front right wheel
    JOINT_RL, // Rear left wheel
    JOINT_RR, // Rear right wheel
    JOINT_SL, // Steering left
    JOINT_SR, // Steering right
    JOINT_COUNT, // Number of joints
  };
  sensor_msgs::JointState joint_state_;
  void publishJointStates(const ros::Time &stamp, const dbw_polaris_msgs::SteeringReport *steering);

  // The signum function: https://stackoverflow.com/questions/1903954/
  template <typename T> static int sgn(T val) {
      return ((T)0 < val) - (val < (T)0);
  }

  // Licensing
  std::string vin_;
  std::string ldate_; // license date
  std::map<uint8_t, std::string> bdate_;

  // Firmware Versions
  PlatformMap firmware_;

  // Frame ID
  std::string frame_id_;

  // Command warnings
  bool warn_cmds_;

  // Pedal LUTs (local/embedded)
  bool pedal_luts_;

  // Ackermann steering
  double acker_wheelbase_;
  double acker_track_;
  double steering_ratio_;
  double wheel_radius_;

  // Subscribed topics
  ros::Subscriber sub_enable_;
  ros::Subscriber sub_disable_;
  ros::Subscriber sub_can_;
  ros::Subscriber sub_brake_;
  ros::Subscriber sub_throttle_;
  ros::Subscriber sub_steering_;
  ros::Subscriber sub_gear_;
  ros::Subscriber sub_calibrate_steering_;

  // Published topics
  ros::Publisher pub_can_;
  ros::Publisher pub_brake_;
  ros::Publisher pub_throttle_;
  ros::Publisher pub_steering_;
  ros::Publisher pub_gear_;
  ros::Publisher pub_imu_;
  ros::Publisher pub_joint_states_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_vin_;
  ros::Publisher pub_sys_enable_;

  // Time Synchronization
  dataspeed_can_msg_filters::ApproximateTime sync_imu_;
};

} // namespace dbw_polaris_can

#endif // _DBW_NODE_H_

