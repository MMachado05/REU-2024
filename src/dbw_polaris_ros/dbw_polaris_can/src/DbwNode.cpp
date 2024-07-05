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

#include "DbwNode.h"
#include <dbw_polaris_can/dispatch.h>
#include <dbw_polaris_can/pedal_lut.h>

// Log once per unique identifier, similar to ROS_LOG_ONCE()
#define ROS_LOG_ONCE_ID(id, level, name, ...) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static std::map<int, bool> map; \
    bool &hit = map[id]; \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && ROS_UNLIKELY(!hit)) \
    { \
      hit = true; \
      ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
    } \
  } while(false)
#define ROS_DEBUG_ONCE_ID(id, ...) ROS_LOG_ONCE_ID(id, ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define ROS_INFO_ONCE_ID(id, ...)  ROS_LOG_ONCE_ID(id, ::ros::console::levels::Info,  ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define ROS_WARN_ONCE_ID(id, ...)  ROS_LOG_ONCE_ID(id, ::ros::console::levels::Warn,  ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define ROS_ERROR_ONCE_ID(id, ...) ROS_LOG_ONCE_ID(id, ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define ROS_FATAL_ONCE_ID(id, ...) ROS_LOG_ONCE_ID(id, ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

namespace dbw_polaris_can
{

// Latest firmware versions
PlatformMap FIRMWARE_LATEST({
  {PlatformVersion(P_POLARIS_GEM,  M_TPEC,  ModuleVersion(1,2,2))},
  {PlatformVersion(P_POLARIS_GEM,  M_STEER, ModuleVersion(1,2,2))},
  {PlatformVersion(P_POLARIS_GEM,  M_BOO,   ModuleVersion(1,2,2))},
  {PlatformVersion(P_POLARIS_RZR,  M_TPEC,  ModuleVersion(0,4,2))},
  {PlatformVersion(P_POLARIS_RZR,  M_STEER, ModuleVersion(0,4,2))},
  {PlatformVersion(P_POLARIS_RZR,  M_BOO,   ModuleVersion(0,4,2))},
});

DbwNode::DbwNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
: sync_imu_(10, boost::bind(&DbwNode::recvCanImu, this, _1), ID_REPORT_ACCEL, ID_REPORT_GYRO)
{
  // Reduce synchronization delay
  sync_imu_.setInterMessageLowerBound(ros::Duration(0.003)); // 10ms period

  // Initialize enable state machine
  prev_enable_ = true;
  enable_ = false;
  override_brake_ = false;
  override_throttle_ = false;
  override_steering_ = false;
  override_gear_ = false;
  fault_brakes_ = false;
  fault_throttle_ = false;
  fault_steering_ = false;
  fault_steering_cal_ = false;
  fault_watchdog_ = false;
  fault_watchdog_using_brakes_ = false;
  fault_watchdog_warned_ = false;
  timeout_brakes_ = false;
  timeout_throttle_ = false;
  timeout_steering_ = false;
  enabled_brakes_ = false;
  enabled_throttle_ = false;
  enabled_steering_ = false;
  gear_warned_ = false;

  // Frame ID
  frame_id_ = "base_footprint";
  priv_nh.getParam("frame_id", frame_id_);

  // Warn on received commands
  warn_cmds_ = true;
  priv_nh.getParam("warn_cmds", warn_cmds_);

  // Pedal LUTs (local/embedded)
  pedal_luts_ = false;
  priv_nh.getParam("pedal_luts", pedal_luts_);

  // Ackermann steering parameters @TODO
  acker_wheelbase_ = 3.08864; // 121.6 inches
  acker_track_ = 1.73228; // 68.2 inches
  steering_ratio_ = 16.2;
  wheel_radius_ = 0.365;
  priv_nh.getParam("ackermann_wheelbase", acker_wheelbase_);
  priv_nh.getParam("ackermann_track", acker_track_);
  priv_nh.getParam("steering_ratio", steering_ratio_);

  // Initialize joint states
  joint_state_.position.resize(JOINT_COUNT);
  joint_state_.velocity.resize(JOINT_COUNT);
  joint_state_.effort.resize(JOINT_COUNT);
  joint_state_.name.resize(JOINT_COUNT);
  joint_state_.name[JOINT_FL] = "wheel_fl"; // Front Left
  joint_state_.name[JOINT_FR] = "wheel_fr"; // Front Right
  joint_state_.name[JOINT_RL] = "wheel_rl"; // Rear Left
  joint_state_.name[JOINT_RR] = "wheel_rr"; // Rear Right
  joint_state_.name[JOINT_SL] = "steer_fl";
  joint_state_.name[JOINT_SR] = "steer_fr";

  // Setup Publishers
  pub_can_ = node.advertise<can_msgs::Frame>("can_tx", 10);
  pub_brake_ = node.advertise<dbw_polaris_msgs::BrakeReport>("brake_report", 2);
  pub_throttle_ = node.advertise<dbw_polaris_msgs::ThrottleReport>("throttle_report", 2);
  pub_steering_ = node.advertise<dbw_polaris_msgs::SteeringReport>("steering_report", 2);
  pub_gear_ = node.advertise<dbw_polaris_msgs::GearReport>("gear_report", 2);
  pub_imu_ = node.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
  pub_joint_states_ = node.advertise<sensor_msgs::JointState>("joint_states", 10);
  pub_twist_ = node.advertise<geometry_msgs::TwistStamped>("twist", 10);
  pub_vin_ = node.advertise<std_msgs::String>("vin", 1, true);
  pub_sys_enable_ = node.advertise<std_msgs::Bool>("dbw_enabled", 1, true);
  publishDbwEnabled();

  // Setup Subscribers
  const ros::TransportHints NODELAY = ros::TransportHints().tcpNoDelay();
  sub_enable_ = node.subscribe("enable", 10, &DbwNode::recvEnable, this, NODELAY);
  sub_disable_ = node.subscribe("disable", 10, &DbwNode::recvDisable, this, NODELAY);
  sub_can_ = node.subscribe("can_rx", 100, &DbwNode::recvCAN, this, NODELAY);
  sub_brake_ = node.subscribe("brake_cmd", 1, &DbwNode::recvBrakeCmd, this, NODELAY);
  sub_throttle_ = node.subscribe("throttle_cmd", 1, &DbwNode::recvThrottleCmd, this, NODELAY);
  sub_steering_ = node.subscribe("steering_cmd", 1, &DbwNode::recvSteeringCmd, this, NODELAY);
  sub_gear_ = node.subscribe("gear_cmd", 1, &DbwNode::recvGearCmd, this, NODELAY);
  sub_calibrate_steering_ = node.subscribe("calibrate_steering", 1, &DbwNode::recvCalibrateSteering, this, NODELAY);

  // Setup Timer
  timer_ = node.createTimer(ros::Duration(1 / 20.0), &DbwNode::timerCallback, this);
}

DbwNode::~DbwNode()
{
}

void DbwNode::recvEnable(const std_msgs::Empty::ConstPtr& msg)
{
  enableSystem();
}

void DbwNode::recvDisable(const std_msgs::Empty::ConstPtr& msg)
{
  disableSystem();
}

void DbwNode::recvCAN(const can_msgs::Frame::ConstPtr& msg)
{
  sync_imu_.processMsg(msg);
  if (!msg->is_rtr && !msg->is_error && !msg->is_extended) {
    switch (msg->id) {
      case ID_BRAKE_REPORT:
        if (msg->dlc >= sizeof(MsgBrakeReport)) {
          const MsgBrakeReport *ptr = (const MsgBrakeReport*)msg->data.elems;
          faultBrakes(ptr->FLT1 || ptr->FLT2);
          faultWatchdog(ptr->FLTWDC, ptr->WDCSRC, ptr->WDCBRK);
          overrideBrake(ptr->OVERRIDE, ptr->TMOUT);
          timeoutBrake(ptr->TMOUT, ptr->ENABLED);
          dbw_polaris_msgs::BrakeReport out;
          out.header.stamp = msg->header.stamp;
          if (ptr->BTYPE == 2 || ptr->BTYPE == 1) {
            // Type 1 is for backwards compatibility only
            out.torque_input = ptr->PI;
            out.torque_cmd = ptr->PC;
            out.torque_output = ptr->PO;
          } else {
            ROS_WARN_THROTTLE(5.0, "Unsupported brake report type: %u", ptr->BTYPE);
          }
          out.enabled = ptr->ENABLED ? true : false;
          out.override = ptr->OVERRIDE ? true : false;
          out.driver = ptr->DRIVER ? true : false;
          out.watchdog_counter.source = ptr->WDCSRC;
          out.watchdog_braking = ptr->WDCBRK ? true : false;
          out.fault_wdc = ptr->FLTWDC ? true : false;
          out.fault_ch1 = ptr->FLT1 ? true : false;
          out.fault_ch2 = ptr->FLT2 ? true : false;
          out.fault_power = ptr->FLTPWR ? true : false;
          out.timeout = ptr->TMOUT ? true : false;
          pub_brake_.publish(out);
          if (ptr->FLT1 || ptr->FLT2 || ptr->FLTPWR) {
            ROS_WARN_THROTTLE(5.0, "Brake fault.    FLT1: %s FLT2: %s FLTPWR: %s",
                ptr->FLT1 ? "true, " : "false,",
                ptr->FLT2 ? "true, " : "false,",
                ptr->FLTPWR ? "true" : "false");
          }
        }
        break;

      case ID_THROTTLE_REPORT:
        if (msg->dlc >= sizeof(MsgThrottleReport)) {
          const MsgThrottleReport *ptr = (const MsgThrottleReport*)msg->data.elems;
          faultThrottle(ptr->FLT1 || ptr->FLT2);
          faultWatchdog(ptr->FLTWDC, ptr->WDCSRC);
          overrideThrottle(ptr->OVERRIDE, ptr->TMOUT);
          timeoutThrottle(ptr->TMOUT, ptr->ENABLED);
          dbw_polaris_msgs::ThrottleReport out;
          out.header.stamp = msg->header.stamp;
          out.pedal_input  = (float)ptr->PI / UINT16_MAX;
          out.pedal_cmd    = (float)ptr->PC / UINT16_MAX;
          out.pedal_output = (float)ptr->PO / UINT16_MAX;
          out.enabled = ptr->ENABLED ? true : false;
          out.override = ptr->OVERRIDE ? true : false;
          out.driver = ptr->DRIVER ? true : false;
          out.watchdog_counter.source = ptr->WDCSRC;
          out.fault_wdc = ptr->FLTWDC ? true : false;
          out.fault_ch1 = ptr->FLT1 ? true : false;
          out.fault_ch2 = ptr->FLT2 ? true : false;
          out.fault_power = ptr->FLTPWR ? true : false;
          out.timeout = ptr->TMOUT ? true : false;
          pub_throttle_.publish(out);
          if (ptr->FLT1 || ptr->FLT2 || ptr->FLTPWR) {
            ROS_WARN_THROTTLE(5.0, "Throttle fault. FLT1: %s FLT2: %s FLTPWR: %s",
                ptr->FLT1 ? "true, " : "false,",
                ptr->FLT2 ? "true, " : "false,",
                ptr->FLTPWR ? "true" : "false");
          }
        }
        break;

      case ID_STEERING_REPORT:
        if (msg->dlc >= sizeof(MsgSteeringReport)) {
          const MsgSteeringReport *ptr = (const MsgSteeringReport*)msg->data.elems;
          faultSteering(ptr->FLTBUS1 || ptr->FLTBUS2);
          faultSteeringCal(ptr->FLTCAL);
          faultWatchdog(ptr->FLTWDC);
          overrideSteering(ptr->OVERRIDE, ptr->TMOUT);
          timeoutSteering(ptr->TMOUT, ptr->ENABLED);
          dbw_polaris_msgs::SteeringReport out;
          out.header.stamp = msg->header.stamp;
          if ((uint16_t)ptr->ANGLE == 0x8000) {
            out.steering_wheel_angle = NAN;
          } else {
            out.steering_wheel_angle = (float)ptr->ANGLE * (float)(0.1 * M_PI / 180);
          }
          out.steering_wheel_cmd_type = ptr->TMODE ? dbw_polaris_msgs::SteeringReport::CMD_TORQUE : dbw_polaris_msgs::SteeringReport::CMD_ANGLE;
          if ((uint16_t)ptr->CMD == 0xC000) {
            out.steering_wheel_cmd = NAN;
          } else if (out.steering_wheel_cmd_type == dbw_polaris_msgs::SteeringReport::CMD_ANGLE) {
            out.steering_wheel_cmd = (float)ptr->CMD * (float)(0.1 * M_PI / 180);
          } else {
            out.steering_wheel_cmd = (float)ptr->CMD / 128.0f;
          }
          if ((uint8_t)ptr->TORQUE == 0x80) {
            out.steering_wheel_torque = NAN;
          } else {
            out.steering_wheel_torque = (float)ptr->TORQUE * (float)0.0625;
          }
          if ((uint16_t)ptr->VEH_VEL == 0x8000) {
            out.speed = NAN;
          } else {
            out.speed = (float)ptr->VEH_VEL * (float)(0.01 / 3.6);
          }
          out.enabled = ptr->ENABLED ? true : false;
          out.override = ptr->OVERRIDE ? true : false;
          out.fault_wdc = ptr->FLTWDC ? true : false;
          out.fault_bus1 = ptr->FLTBUS1 ? true : false;
          out.fault_bus2 = ptr->FLTBUS2 ? true : false;
          out.fault_calibration = ptr->FLTCAL ? true : false;
          out.fault_power = ptr->FLTPWR ? true : false;
          out.timeout = ptr->TMOUT ? true : false;
          pub_steering_.publish(out);
          geometry_msgs::TwistStamped twist;
          twist.header.stamp = out.header.stamp;
          twist.header.frame_id = frame_id_;
          twist.twist.linear.x = out.speed;
          twist.twist.angular.z = out.speed * tan(out.steering_wheel_angle / steering_ratio_) / acker_wheelbase_;
          pub_twist_.publish(twist);
          publishJointStates(msg->header.stamp, &out);
          if (ptr->FLTBUS1 || ptr->FLTBUS2 || ptr->FLTPWR) {
            ROS_WARN_THROTTLE(5.0, "Steering fault. FLT1: %s FLT2: %s FLTPWR: %s",
                ptr->FLTBUS1 ? "true, " : "false,",
                ptr->FLTBUS2 ? "true, " : "false,",
                ptr->FLTPWR  ? "true" : "false");
            if (ptr->FLTBUS2) {
              ROS_WARN_THROTTLE(5.0, "Steering fault. Too many calibrations stored. Reset need to continue");
            }
          } else if (ptr->FLTCAL) {
            ROS_WARN_THROTTLE(5.0, "Steering calibration fault. Press the two steering multiplier buttons at the same "
                                   "time to set the center offset when the wheel is straight. For a more exact "
                                   "calibration set the SteeringCal and SteeringCal offset parameters using DbwConfig.");
          }
        }
        break;

      case ID_GEAR_REPORT:
        if (msg->dlc >= sizeof(MsgGearReport)) {
          const MsgGearReport *ptr = (const MsgGearReport*)msg->data.elems;
          overrideGear(ptr->OVERRIDE);
          dbw_polaris_msgs::GearReport out;
          out.header.stamp = msg->header.stamp;
          out.state.gear = ptr->STATE;
          out.cmd.gear = ptr->CMD;
          out.override = ptr->OVERRIDE ? true : false;
          out.fault_bus = ptr->FLTBUS ? true : false;
          out.reject.value = ptr->REJECT;
          if (out.reject.value == dbw_polaris_msgs::GearReject::NONE) {
            gear_warned_ = false;
          } else if (!gear_warned_) {
            gear_warned_ = true;
            switch (out.reject.value) {
              case dbw_polaris_msgs::GearReject::SHIFT_IN_PROGRESS:
                ROS_WARN("Gear shift rejected: Shift in progress");
                break;
              case dbw_polaris_msgs::GearReject::OVERRIDE:
                ROS_WARN("Gear shift rejected: Override on brake, throttle, or steering");
                break;
              case dbw_polaris_msgs::GearReject::NEUTRAL:
                ROS_WARN("Gear shift rejected: Manually shift to neutral before auto-shift");
                break;
              case dbw_polaris_msgs::GearReject::VEHICLE:
                ROS_WARN("Gear shift rejected: Rejected by vehicle, try pressing the brakes");
                break;
              case dbw_polaris_msgs::GearReject::UNSUPPORTED:
                ROS_WARN("Gear shift rejected: Unsupported gear command");
                break;
              case dbw_polaris_msgs::GearReject::FAULT:
                ROS_WARN("Gear shift rejected: System in fault state");
                break;
            }
          }
          pub_gear_.publish(out);
        }
        break;

      case ID_LICENSE:
        if (msg->dlc >= sizeof(MsgLicense)) {
          const MsgLicense *ptr = (const MsgLicense*)msg->data.elems;
          const Module module = ptr->module ? (Module)ptr->module : M_STEER; // Legacy steering firmware reports zero for module
          const char * str_m = moduleToString(module);
          ROS_DEBUG("LICENSE(%x,%02X,%s)", ptr->module, ptr->mux, str_m);
          if (ptr->ready) {
            ROS_INFO_ONCE_ID(module, "Licensing: %s ready", str_m);
            if (ptr->trial) {
              ROS_WARN_ONCE_ID(module, "Licensing: %s one or more features licensed as a counted trial. Visit https://www.dataspeedinc.com/products/maintenance-subscription/ to request a full license.", str_m);
            }
            if (ptr->expired) {
              ROS_WARN_ONCE_ID(module, "Licensing: %s one or more feature licenses expired due to the firmware build date", str_m);
            }
          } else if (module == M_STEER) {
            ROS_INFO_THROTTLE(10.0, "Licensing: Waiting for VIN...");
          } else {
            ROS_INFO_THROTTLE(10.0, "Licensing: Waiting for required info...");
          }
          if (ptr->mux == LIC_MUX_LDATE0) {
            if (ldate_.size() == 0) {
              ldate_.push_back(ptr->ldate0.ldate0);
              ldate_.push_back(ptr->ldate0.ldate1);
              ldate_.push_back(ptr->ldate0.ldate2);
              ldate_.push_back(ptr->ldate0.ldate3);
              ldate_.push_back(ptr->ldate0.ldate4);
              ldate_.push_back(ptr->ldate0.ldate5);
            }
          } else if (ptr->mux == LIC_MUX_LDATE1) {
            if (ldate_.size() == 6) {
              ldate_.push_back(ptr->ldate1.ldate6);
              ldate_.push_back(ptr->ldate1.ldate7);
              ldate_.push_back(ptr->ldate1.ldate8);
              ldate_.push_back(ptr->ldate1.ldate9);
              ROS_INFO("Licensing: %s license string date: %s", str_m, ldate_.c_str());
            }
          } else if (ptr->mux == LIC_MUX_MAC) {
            ROS_INFO_ONCE("Licensing: %s MAC: %02X:%02X:%02X:%02X:%02X:%02X", str_m,
                          ptr->mac.addr0, ptr->mac.addr1,
                          ptr->mac.addr2, ptr->mac.addr3,
                          ptr->mac.addr4, ptr->mac.addr5);
          } else if (ptr->mux == LIC_MUX_BDATE0) {
            std::string &bdate = bdate_[module];
            if (bdate.size() == 0) {
              bdate.push_back(ptr->bdate0.date0);
              bdate.push_back(ptr->bdate0.date1);
              bdate.push_back(ptr->bdate0.date2);
              bdate.push_back(ptr->bdate0.date3);
              bdate.push_back(ptr->bdate0.date4);
              bdate.push_back(ptr->bdate0.date5);
            }
          } else if (ptr->mux == LIC_MUX_BDATE1) {
            std::string &bdate = bdate_[module];
            if (bdate.size() == 6) {
              bdate.push_back(ptr->bdate1.date6);
              bdate.push_back(ptr->bdate1.date7);
              bdate.push_back(ptr->bdate1.date8);
              bdate.push_back(ptr->bdate1.date9);
              ROS_INFO("Licensing: %s firmware build date: %s", str_m, bdate.c_str());
            }
          } else if (ptr->mux == LIC_MUX_VIN0) {
            if (vin_.size() == 0) {
              vin_.push_back(ptr->vin0.vin00);
              vin_.push_back(ptr->vin0.vin01);
              vin_.push_back(ptr->vin0.vin02);
              vin_.push_back(ptr->vin0.vin03);
              vin_.push_back(ptr->vin0.vin04);
              vin_.push_back(ptr->vin0.vin05);
            }
          } else if (ptr->mux == LIC_MUX_VIN1) {
            if (vin_.size() == 6) {
              vin_.push_back(ptr->vin1.vin06);
              vin_.push_back(ptr->vin1.vin07);
              vin_.push_back(ptr->vin1.vin08);
              vin_.push_back(ptr->vin1.vin09);
              vin_.push_back(ptr->vin1.vin10);
              vin_.push_back(ptr->vin1.vin11);
            }
          } else if (ptr->mux == LIC_MUX_VIN2) {
            if (vin_.size() == 12) {
              vin_.push_back(ptr->vin2.vin12);
              vin_.push_back(ptr->vin2.vin13);
              vin_.push_back(ptr->vin2.vin14);
              vin_.push_back(ptr->vin2.vin15);
              vin_.push_back(ptr->vin2.vin16);
              std_msgs::String msg; msg.data = vin_;
              pub_vin_.publish(msg);
              ROS_INFO("Licensing: VIN: %s", vin_.c_str());
            }
          } else if ((LIC_MUX_F0 <= ptr->mux) && (ptr->mux <= LIC_MUX_F7)) {
            constexpr std::array<const char*, 8> NAME = {"BASE","CONTROL","SENSORS","","","","",""};
            constexpr std::array<bool, 8> WARN = {true, true, true, false, true, true, true, true};
            const size_t i = ptr->mux - LIC_MUX_F0;
            const int id = module * NAME.size() + i;
            const std::string name = strcmp(NAME[i], "") ? NAME[i] : std::string(1, '0' + i);
            if (ptr->license.enabled) {
              ROS_INFO_ONCE_ID(id, "Licensing: %s feature '%s' enabled%s", str_m, name.c_str(), ptr->license.trial ? " as a counted trial" : "");
            } else if (ptr->ready && !WARN[i]) {
              ROS_INFO_ONCE_ID(id, "Licensing: %s feature '%s' not licensed.", str_m, name.c_str());
            } else if (ptr->ready) {
              ROS_WARN_ONCE_ID(id, "Licensing: %s feature '%s' not licensed. Visit https://www.dataspeedinc.com/products/maintenance-subscription/ to request a license.", str_m, name.c_str());
            }
            if (ptr->ready && (module == M_STEER) && (ptr->license.trial || (!ptr->license.enabled && WARN[i]))) {
              ROS_INFO_ONCE("Licensing: Feature '%s' trials used: %u, remaining: %u", name.c_str(),
                            ptr->license.trials_used, ptr->license.trials_left);
            }
          }
        }
        break;

      case ID_VERSION:
        if (msg->dlc >= sizeof(MsgVersion)) {
          const MsgVersion *ptr = (const MsgVersion*)msg->data.elems;
          const PlatformVersion version((Platform)ptr->platform, (Module)ptr->module, ptr->major, ptr->minor, ptr->build);
          const ModuleVersion latest = FIRMWARE_LATEST.findModule(version);
          const char * str_p = platformToString(version.p);
          const char * str_m = moduleToString(version.m);
          if (firmware_.findModule(version) != version.v) {
            firmware_.insert(version);
            if (latest.valid()) {
              ROS_INFO("Detected %s %s firmware version %u.%u.%u", str_p, str_m, ptr->major, ptr->minor, ptr->build);
            } else {
              ROS_WARN("Detected %s %s firmware version %u.%u.%u, which is unsupported. Platform: 0x%02X, Module: %u", str_p, str_m,
                       ptr->major, ptr->minor, ptr->build, ptr->platform, ptr->module);
            }
            if (version < latest) {
              ROS_WARN("Firmware %s %s has old  version %u.%u.%u, updating to %u.%u.%u is suggested.", str_p, str_m,
                       version.v.major(), version.v.minor(), version.v.build(),
                       latest.major(),  latest.minor(),  latest.build());
            }
          }
        }
        break;

      case ID_BRAKE_CMD:
        ROS_WARN_COND(warn_cmds_ && !((const MsgBrakeCmd*)msg->data.elems)->RES1 && !((const MsgBrakeCmd*)msg->data.elems)->RES2,
                                  "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Brake. Id: 0x%03X", ID_BRAKE_CMD);
        break;
      case ID_THROTTLE_CMD:
        ROS_WARN_COND(warn_cmds_ && !((const MsgThrottleCmd*)msg->data.elems)->RES1 && !((const MsgThrottleCmd*)msg->data.elems)->RES2,
                                  "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Throttle. Id: 0x%03X", ID_THROTTLE_CMD);
        break;
      case ID_STEERING_CMD:
        ROS_WARN_COND(warn_cmds_ && !((const MsgSteeringCmd*)msg->data.elems)->RES1 && !((const MsgSteeringCmd*)msg->data.elems)->RES2,
                                  "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Steering. Id: 0x%03X", ID_STEERING_CMD);
        break;
      case ID_GEAR_CMD:
        ROS_WARN_COND(warn_cmds_ && !((const MsgGearCmd*)msg->data.elems)->RES1 && !((const MsgGearCmd*)msg->data.elems)->RES2,
                                  "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Shifting. Id: 0x%03X", ID_GEAR_CMD);
        break;

      case 0x100 ... 0x103: // DBW2 steer/brake/throttle/gear report
      case 0x6C0 ... 0x6C5: // DBW2 ECU info for each module
        ROS_WARN_ONCE_ID(msg->id, "Received unsupported CAN ID %03X from next-generation drive-by-wire system (DBW2)"
                                  "\nUse the ROS2 ds_dbw_can package instead", msg->id);
        break;
    }
  }
#if 0
  ROS_INFO("ena: %s, clr: %s, brake: %s, throttle: %s, steering: %s, gear: %s",
           enabled() ? "true " : "false",
           clear() ? "true " : "false",
           override_brake_ ? "true " : "false",
           override_throttle_ ? "true " : "false",
           override_steering_ ? "true " : "false",
           override_gear_ ? "true " : "false"
       );
#endif
}

void DbwNode::recvCanImu(const std::vector<can_msgs::Frame::ConstPtr> &msgs) {
  ROS_ASSERT(msgs.size() == 2);
  ROS_ASSERT(msgs[0]->id == ID_REPORT_ACCEL);
  ROS_ASSERT(msgs[1]->id == ID_REPORT_GYRO);
  if ((msgs[0]->dlc >= sizeof(MsgReportAccel)) && (msgs[1]->dlc >= sizeof(MsgReportGyro))) {
    const MsgReportAccel *ptr_accel = (const MsgReportAccel*)msgs[0]->data.elems;
    const MsgReportGyro *ptr_gyro = (const MsgReportGyro*)msgs[1]->data.elems;
    sensor_msgs::Imu out;
    out.header.stamp = msgs[0]->header.stamp;
    out.header.frame_id = frame_id_;
    out.orientation_covariance[0] = -1; // Orientation not present
    if ((uint16_t)ptr_accel->accel_long == 0x8000) {
      out.linear_acceleration.x = NAN;
    } else {
      out.linear_acceleration.x = (double)ptr_accel->accel_long * 0.01;
    }
    if ((uint16_t)ptr_accel->accel_lat == 0x8000) {
      out.linear_acceleration.y = NAN;
    } else {
      out.linear_acceleration.y = (double)ptr_accel->accel_lat * -0.01;
    }
    if ((uint16_t)ptr_accel->accel_vert == 0x8000) {
      out.linear_acceleration.z = NAN;
    } else {
      out.linear_acceleration.z = (double)ptr_accel->accel_vert * -0.01;
    }
    if ((uint16_t)ptr_gyro->gyro_roll == 0x8000) {
      out.angular_velocity.x = NAN;
    } else {
      out.angular_velocity.x = (double)ptr_gyro->gyro_roll * 0.0002;
    }
    if ((uint16_t)ptr_gyro->gyro_pitch == 0x8000) {
      out.angular_velocity.y = NAN;
    } else {
      out.angular_velocity.y = (double)ptr_gyro->gyro_pitch * 0.0002;
    }
    if ((uint16_t)ptr_gyro->gyro_yaw == 0x8000) {
      out.angular_velocity.z = NAN;
    } else {
      out.angular_velocity.z = (double)ptr_gyro->gyro_yaw * 0.0002;
    }
    pub_imu_.publish(out);
  }
#if 0
  ROS_INFO("Time: %u.%u, %u.%u, delta: %fms",
           msgs[0]->header.stamp.sec, msgs[0]->header.stamp.nsec,
           msgs[1]->header.stamp.sec, msgs[1]->header.stamp.nsec,
           labs((msgs[1]->header.stamp - msgs[0]->header.stamp).toNSec()) / 1000000.0);
#endif
}

void DbwNode::recvBrakeCmd(const dbw_polaris_msgs::BrakeCmd::ConstPtr& msg)
{
  can_msgs::Frame out;
  out.id = ID_BRAKE_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgBrakeCmd);
  MsgBrakeCmd *ptr = (MsgBrakeCmd*)out.data.elems;
  memset(ptr, 0x00, sizeof(*ptr));
  switch (msg->pedal_cmd_type) {
    case dbw_polaris_msgs::BrakeCmd::CMD_NONE:
      break;
    case dbw_polaris_msgs::BrakeCmd::CMD_PERCENT:
      ptr->CMD_TYPE = dbw_polaris_msgs::BrakeCmd::CMD_PERCENT;
      ptr->PCMD = std::clamp<float>(msg->pedal_cmd * UINT16_MAX, 0, UINT16_MAX);
      break;
    case dbw_polaris_msgs::BrakeCmd::CMD_TORQUE:
      ptr->CMD_TYPE = dbw_polaris_msgs::BrakeCmd::CMD_TORQUE;
      ptr->PCMD = std::clamp<float>(msg->pedal_cmd, 0, UINT16_MAX);
      break;
    case dbw_polaris_msgs::BrakeCmd::CMD_TORQUE_RQ:
      ptr->CMD_TYPE = dbw_polaris_msgs::BrakeCmd::CMD_TORQUE_RQ;
      ptr->PCMD = std::clamp<float>(msg->pedal_cmd, 0, UINT16_MAX);
      break;
    default:
      ROS_WARN("Unknown brake command type: %u", msg->pedal_cmd_type);
      break;
  }
  if (enabled() && msg->enable) {
    ptr->EN = 1;
  }
  if (clear() || msg->clear) {
    ptr->CLEAR = 1;
  }
  if (msg->ignore) {
    ptr->IGNORE = 1;
  }
  ptr->COUNT = msg->count;
  pub_can_.publish(out);
}

void DbwNode::recvThrottleCmd(const dbw_polaris_msgs::ThrottleCmd::ConstPtr& msg)
{
  can_msgs::Frame out;
  out.id = ID_THROTTLE_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgThrottleCmd);
  MsgThrottleCmd *ptr = (MsgThrottleCmd*)out.data.elems;
  memset(ptr, 0x00, sizeof(*ptr));
  bool fwd = !pedal_luts_; // Forward command type, or apply pedal LUTs locally
  float cmd = 0.0;
  switch (msg->pedal_cmd_type) {
    case dbw_polaris_msgs::ThrottleCmd::CMD_NONE:
      break;
    case dbw_polaris_msgs::ThrottleCmd::CMD_PEDAL:
      ptr->CMD_TYPE = dbw_polaris_msgs::ThrottleCmd::CMD_PEDAL;
      cmd = msg->pedal_cmd;
      break;
    case dbw_polaris_msgs::ThrottleCmd::CMD_PERCENT:
      if (fwd) {
        ptr->CMD_TYPE = dbw_polaris_msgs::ThrottleCmd::CMD_PERCENT;
        cmd = msg->pedal_cmd;
      } else {
        ptr->CMD_TYPE = dbw_polaris_msgs::ThrottleCmd::CMD_PEDAL;
        cmd = throttlePedalFromPercent(msg->pedal_cmd);
      }
      break;
    default:
      ROS_WARN("Unknown throttle command type: %u", msg->pedal_cmd_type);
      break;
  }
  ptr->PCMD = std::clamp<float>(cmd * UINT16_MAX, 0, UINT16_MAX);
  if (enabled() && msg->enable) {
    ptr->EN = 1;
  }
  if (clear() || msg->clear) {
    ptr->CLEAR = 1;
  }
  if (msg->ignore) {
    ptr->IGNORE = 1;
  }
  ptr->COUNT = msg->count;
  pub_can_.publish(out);
}

void DbwNode::recvSteeringCmd(const dbw_polaris_msgs::SteeringCmd::ConstPtr& msg)
{
  can_msgs::Frame out;
  out.id = ID_STEERING_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgSteeringCmd);
  MsgSteeringCmd *ptr = (MsgSteeringCmd*)out.data.elems;
  memset(ptr, 0x00, sizeof(*ptr));
  switch (msg->cmd_type) {
    case dbw_polaris_msgs::SteeringCmd::CMD_ANGLE:
      ptr->SCMD = std::clamp<float>(msg->steering_wheel_angle_cmd * (float)(180 / M_PI * 10), -INT16_MAX, INT16_MAX);
      if (fabsf(msg->steering_wheel_angle_velocity) > 0) {
        ptr->SVEL = std::clamp<float>(roundf(fabsf(msg->steering_wheel_angle_velocity) * (float)(180 / M_PI / 4)), 1, 254);
      }
      ptr->CMD_TYPE = dbw_polaris_msgs::SteeringCmd::CMD_ANGLE;
      break;
    case dbw_polaris_msgs::SteeringCmd::CMD_TORQUE:
      ptr->SCMD = std::clamp<float>(msg->steering_wheel_torque_cmd * 128, -INT16_MAX, INT16_MAX);
      ptr->CMD_TYPE = dbw_polaris_msgs::SteeringCmd::CMD_TORQUE;
      break;
    default:
      ROS_WARN("Unknown steering command type: %u", msg->cmd_type);
      break;
  }
  if (enabled() && msg->enable) {
    ptr->EN = 1;
  }
  if (clear() || msg->clear) {
    ptr->CLEAR = 1;
  }
  if (msg->ignore) {
    ptr->IGNORE = 1;
  }
  if (msg->calibrate) {
    ptr->CAL = 1;
  }
  if (msg->quiet) {
    ptr->QUIET = 1;
  }
  if (msg->alert) {
    ptr->ALERT = 1;
  }
  ptr->COUNT = msg->count;
  pub_can_.publish(out);
}

void DbwNode::recvGearCmd(const dbw_polaris_msgs::GearCmd::ConstPtr& msg)
{
  can_msgs::Frame out;
  out.id = ID_GEAR_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgGearCmd);
  MsgGearCmd *ptr = (MsgGearCmd*)out.data.elems;
  memset(ptr, 0x00, sizeof(*ptr));
  if (enabled()) {
    ptr->GCMD = msg->cmd.gear;
  }
  if (clear() || msg->clear) {
    ptr->CLEAR = 1;
  }
  pub_can_.publish(out);
}

void DbwNode::recvCalibrateSteering(const std_msgs::Empty::ConstPtr& msg)
{
  /* Send steering command to save current angle as zero.
   * The preferred method is to set the 'calibrate' field in a ROS steering
   * command so that recvSteeringCmd() saves the current angle as the
   * specified command.
   */
  can_msgs::Frame out;
  out.id = ID_STEERING_CMD;
  out.is_extended = false;
  out.dlc = 4; // Sending the full eight bytes will fault the watchdog counter (if enabled)
  MsgSteeringCmd *ptr = (MsgSteeringCmd*)out.data.elems;
  ptr->CAL = 1;
  pub_can_.publish(out);
}

bool DbwNode::publishDbwEnabled(bool force)
{
  bool en = enabled();
  bool change = prev_enable_ != en;
  if (change || force) {
    std_msgs::Bool msg;
    msg.data = en;
    pub_sys_enable_.publish(msg);
  }
  prev_enable_ = en;
  return change;
}

void DbwNode::timerCallback(const ros::TimerEvent& event)
{
  // Publish status periodically, in addition to latched and on change
  if (publishDbwEnabled(true)) {
    ROS_WARN("DBW system enable status changed unexpectedly");
  }

  // Clear override statuses if necessary
  if (clear()) {
    can_msgs::Frame out;
    out.is_extended = false;

    if (override_brake_) {
      out.id = ID_BRAKE_CMD;
      out.dlc = 4; // Sending the full eight bytes will fault the watchdog counter (if enabled)
      memset(out.data.elems, 0x00, 8);
      ((MsgBrakeCmd*)out.data.elems)->CLEAR = 1;
      pub_can_.publish(out);
    }

    if (override_throttle_) {
      out.id = ID_THROTTLE_CMD;
      out.dlc = 4; // Sending the full eight bytes will fault the watchdog counter (if enabled)
      memset(out.data.elems, 0x00, 8);
      ((MsgThrottleCmd*)out.data.elems)->CLEAR = 1;
      pub_can_.publish(out);
    }

    if (override_steering_) {
      out.id = ID_STEERING_CMD;
      out.dlc = 4; // Sending the full eight bytes will fault the watchdog counter (if enabled)
      memset(out.data.elems, 0x00, 8);
      ((MsgSteeringCmd*)out.data.elems)->CLEAR = 1;
      pub_can_.publish(out);
    }

    if (override_gear_) {
      out.id = ID_GEAR_CMD;
      out.dlc = sizeof(MsgGearCmd);
      memset(out.data.elems, 0x00, 8);
      ((MsgGearCmd*)out.data.elems)->CLEAR = 1;
      pub_can_.publish(out);
    }
  }
}

void DbwNode::enableSystem()
{
  if (!enable_) {
    if (fault()) {
      if (fault_steering_cal_) {
        ROS_WARN("DBW system not enabled. Steering calibration fault.");
      }
      if (fault_brakes_) {
        ROS_WARN("DBW system not enabled. Braking fault.");
      }
      if (fault_throttle_) {
        ROS_WARN("DBW system not enabled. Throttle fault.");
      }
      if (fault_steering_) {
        ROS_WARN("DBW system not enabled. Steering fault.");
      }
      if (fault_watchdog_) {
        ROS_WARN("DBW system not enabled. Watchdog fault.");
      }
    } else {
      enable_ = true;
      if (publishDbwEnabled()) {
        ROS_INFO("DBW system enabled.");
      } else {
        ROS_INFO("DBW system enable requested. Waiting for ready.");
      }
    }
  }
}

void DbwNode::disableSystem()
{
  if (enable_) {
    enable_ = false;
    publishDbwEnabled();
    ROS_WARN("DBW system disabled.");
  }
}

void DbwNode::buttonCancel()
{
  if (enable_) {
    enable_ = false;
    publishDbwEnabled();
    ROS_WARN("DBW system disabled. Cancel button pressed.");
  }
}

void DbwNode::overrideBrake(bool override, bool timeout)
{
  bool en = enabled();
  if (en && timeout) {
    override = false;
  }
  if (en && override) {
    enable_ = false;
  }
  override_brake_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_WARN("DBW system disabled. Driver override on brake/throttle pedal.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::overrideThrottle(bool override, bool timeout)
{
  bool en = enabled();
  if (en && timeout) {
    override = false;
  }
  if (en && override) {
    enable_ = false;
  }
  override_throttle_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_WARN("DBW system disabled. Driver override on brake/throttle pedal.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::overrideSteering(bool override, bool timeout)
{
  bool en = enabled();
  if (en && timeout) {
    override = false;
  }
  if (en && override) {
    enable_ = false;
  }
  override_steering_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_WARN("DBW system disabled. Driver override on steering wheel.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::overrideGear(bool override)
{
  bool en = enabled();
  if (en && override) {
    enable_ = false;
  }
  override_gear_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_WARN("DBW system disabled. Driver override on shifter.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::timeoutBrake(bool timeout, bool enabled)
{
  if (!timeout_brakes_ && enabled_brakes_ && timeout && !enabled) {
    ROS_WARN("Brake subsystem disabled after 100ms command timeout");
  }
  timeout_brakes_ = timeout;
  enabled_brakes_ = enabled;
}

void DbwNode::timeoutThrottle(bool timeout, bool enabled)
{
  if (!timeout_throttle_ && enabled_throttle_ && timeout && !enabled) {
    ROS_WARN("Throttle subsystem disabled after 100ms command timeout");
  }
  timeout_throttle_ = timeout;
  enabled_throttle_ = enabled;
}

void DbwNode::timeoutSteering(bool timeout, bool enabled)
{
  if (!timeout_steering_ && enabled_steering_ && timeout && !enabled) {
    ROS_WARN("Steering subsystem disabled after 100ms command timeout");
  }
  timeout_steering_ = timeout;
  enabled_steering_ = enabled;
}

void DbwNode::faultBrakes(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_brakes_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Braking fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::faultThrottle(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_throttle_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Throttle fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::faultSteering(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_steering_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Steering fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::faultSteeringCal(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_steering_cal_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Steering calibration fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::faultWatchdog(bool fault, uint8_t src, bool braking)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_watchdog_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Watchdog fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
  if (braking && !fault_watchdog_using_brakes_) {
    ROS_WARN("Watchdog event: Alerting driver and applying brakes.");
  } else if (!braking && fault_watchdog_using_brakes_) {
    ROS_INFO("Watchdog event: Driver has successfully taken control.");
  }
  if (fault && src && !fault_watchdog_warned_) {
      switch (src) {
        case dbw_polaris_msgs::WatchdogCounter::OTHER_BRAKE:
          ROS_WARN("Watchdog event: Fault determined by brake controller");
          break;
        case dbw_polaris_msgs::WatchdogCounter::OTHER_THROTTLE:
          ROS_WARN("Watchdog event: Fault determined by throttle controller");
          break;
        case dbw_polaris_msgs::WatchdogCounter::OTHER_STEERING:
          ROS_WARN("Watchdog event: Fault determined by steering controller");
          break;
        case dbw_polaris_msgs::WatchdogCounter::BRAKE_COUNTER:
          ROS_WARN("Watchdog event: Brake command counter failed to increment");
          break;
        case dbw_polaris_msgs::WatchdogCounter::BRAKE_DISABLED:
          ROS_WARN("Watchdog event: Brake transition to disabled while in gear or moving");
          break;
        case dbw_polaris_msgs::WatchdogCounter::BRAKE_COMMAND:
          ROS_WARN("Watchdog event: Brake command timeout after 100ms");
          break;
        case dbw_polaris_msgs::WatchdogCounter::BRAKE_REPORT:
          ROS_WARN("Watchdog event: Brake report timeout after 100ms");
          break;
        case dbw_polaris_msgs::WatchdogCounter::THROTTLE_COUNTER:
          ROS_WARN("Watchdog event: Throttle command counter failed to increment");
          break;
        case dbw_polaris_msgs::WatchdogCounter::THROTTLE_DISABLED:
          ROS_WARN("Watchdog event: Throttle transition to disabled while in gear or moving");
          break;
        case dbw_polaris_msgs::WatchdogCounter::THROTTLE_COMMAND:
          ROS_WARN("Watchdog event: Throttle command timeout after 100ms");
          break;
        case dbw_polaris_msgs::WatchdogCounter::THROTTLE_REPORT:
          ROS_WARN("Watchdog event: Throttle report timeout after 100ms");
          break;
        case dbw_polaris_msgs::WatchdogCounter::STEERING_COUNTER:
          ROS_WARN("Watchdog event: Steering command counter failed to increment");
          break;
        case dbw_polaris_msgs::WatchdogCounter::STEERING_DISABLED:
          ROS_WARN("Watchdog event: Steering transition to disabled while in gear or moving");
          break;
        case dbw_polaris_msgs::WatchdogCounter::STEERING_COMMAND:
          ROS_WARN("Watchdog event: Steering command timeout after 100ms");
          break;
        case dbw_polaris_msgs::WatchdogCounter::STEERING_REPORT:
          ROS_WARN("Watchdog event: Steering report timeout after 100ms");
          break;
      }
      fault_watchdog_warned_ = true;
  } else if (!fault) {
    fault_watchdog_warned_ = false;
  }
  fault_watchdog_using_brakes_ = braking;
  if (fault && !fault_watchdog_using_brakes_ && fault_watchdog_warned_) {
    ROS_WARN_THROTTLE(2.0, "Watchdog event: Press left OK button on the steering wheel or cycle power to clear event.");
  }
}

void DbwNode::faultWatchdog(bool fault, uint8_t src) {
  faultWatchdog(fault, src, fault_watchdog_using_brakes_); // No change to 'using brakes' status
}

void DbwNode::publishJointStates(const ros::Time &stamp, const dbw_polaris_msgs::SteeringReport *steering)
{
  double dt = (stamp - joint_state_.header.stamp).toSec();
  if (steering) {
    if (std::isfinite(steering->steering_wheel_angle)) {
      const double L = acker_wheelbase_;
      const double W = acker_track_;
      const double r = L / tan(steering->steering_wheel_angle / steering_ratio_);
      joint_state_.position[JOINT_SL] = atan(L / (r - W/2));
      joint_state_.position[JOINT_SR] = atan(L / (r + W/2));
    }
    if (std::isfinite(steering->speed)) {
      joint_state_.velocity[JOINT_FL] = steering->speed / wheel_radius_;
      joint_state_.velocity[JOINT_FR] = steering->speed / wheel_radius_;
      joint_state_.velocity[JOINT_RL] = steering->speed / wheel_radius_;
      joint_state_.velocity[JOINT_RR] = steering->speed / wheel_radius_;
    }
  }
  if (dt < 0.5) {
    for (size_t i = JOINT_FL; i <= JOINT_RR; i++) {
      joint_state_.position[i] = fmod(joint_state_.position[i] + dt * joint_state_.velocity[i], 2*M_PI);
    }
  }
  joint_state_.header.stamp = stamp;
  pub_joint_states_.publish(joint_state_);
}

} // namespace dbw_polaris_can

