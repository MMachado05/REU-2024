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

#ifndef _DBW_POLARIS_CAN_DISPATCH_H
#define _DBW_POLARIS_CAN_DISPATCH_H
#include <stdint.h>

namespace dbw_polaris_can
{

typedef struct {
  uint16_t PCMD;
  uint8_t :4;
  uint8_t CMD_TYPE :4;
  uint8_t EN :1;
  uint8_t CLEAR :1;
  uint8_t IGNORE :1;
  uint8_t :3;
  uint8_t RES2 :1;
  uint8_t RES1 :1;
  uint8_t :8;
  uint8_t :8;
  uint8_t :8;
  uint8_t COUNT;
} MsgBrakeCmd;

typedef struct {
  uint16_t PI;
  uint16_t PC;
  uint16_t PO;
  uint8_t BTYPE :2;
  uint8_t :1;
  uint8_t WDCBRK :1;
  uint8_t WDCSRC :4;
  uint8_t ENABLED :1;
  uint8_t OVERRIDE :1;
  uint8_t DRIVER :1;
  uint8_t FLTWDC :1;
  uint8_t FLT1 :1;
  uint8_t FLT2 :1;
  uint8_t FLTPWR :1;
  uint8_t TMOUT :1;
} MsgBrakeReport;

typedef struct {
  uint16_t PCMD;
  uint8_t :4;
  uint8_t CMD_TYPE :4;
  uint8_t EN :1;
  uint8_t CLEAR :1;
  uint8_t IGNORE :1;
  uint8_t :3;
  uint8_t RES2 :1;
  uint8_t RES1 :1;
  uint8_t :8;
  uint8_t :8;
  uint8_t :8;
  uint8_t COUNT;
} MsgThrottleCmd;

typedef struct {
  uint16_t PI;
  uint16_t PC;
  uint16_t PO;
  uint8_t :4;
  uint8_t WDCSRC :4;
  uint8_t ENABLED :1;
  uint8_t OVERRIDE :1;
  uint8_t DRIVER :1;
  uint8_t FLTWDC :1;
  uint8_t FLT1 :1;
  uint8_t FLT2 :1;
  uint8_t FLTPWR :1;
  uint8_t TMOUT :1;
} MsgThrottleReport;

typedef struct {
  int16_t SCMD;
  uint8_t EN :1;
  uint8_t CLEAR :1;
  uint8_t IGNORE :1;
  uint8_t CAL :1;
  uint8_t QUIET :1;
  uint8_t RES1 :1;
  uint8_t ALERT :1;
  uint8_t CMD_TYPE :1;
  uint8_t SVEL;
  uint8_t RES2 :1;
  uint8_t :7;
  uint8_t :8;
  uint8_t :8;
  uint8_t COUNT;
} MsgSteeringCmd;

typedef struct {
  int16_t ANGLE;
  int16_t CMD :15;
  uint8_t TMODE :1; // Torque mode
  int16_t VEH_VEL;
  int8_t TORQUE;
  uint8_t ENABLED :1;
  uint8_t OVERRIDE :1;
  uint8_t FLTPWR :1;
  uint8_t FLTWDC :1;
  uint8_t FLTBUS1 :1;
  uint8_t FLTBUS2 :1;
  uint8_t FLTCAL :1;
  uint8_t TMOUT :1;
} MsgSteeringReport;

typedef struct {
  uint8_t GCMD :3;
  uint8_t :2;
  uint8_t RES2 :1;
  uint8_t RES1 :1;
  uint8_t CLEAR :1;
} MsgGearCmd;

typedef struct {
  uint8_t STATE :3;
  uint8_t OVERRIDE :1;
  uint8_t CMD :3;
  uint8_t FLTBUS :1;
  uint8_t REJECT :3;
  uint8_t :5;
} MsgGearReport;

typedef struct {
  int16_t accel_lat;
  int16_t accel_long;
  int16_t accel_vert;
} MsgReportAccel;

typedef struct {
  int16_t gyro_roll;
  int16_t gyro_yaw;
  int16_t gyro_pitch;
} MsgReportGyro;

typedef enum {
  LIC_MUX_F0     = 0x00, // Feature 0 (BASE)
  LIC_MUX_F1     = 0x01, // Feature 1 (CONTROL)
  LIC_MUX_F2     = 0x02, // Feature 2 (SENSORS)
  LIC_MUX_F3     = 0x03, // Feature 3 (unused)
  LIC_MUX_F4     = 0x04, // Feature 4 (unused)
  LIC_MUX_F5     = 0x05, // Feature 5 (unused)
  LIC_MUX_F6     = 0x06, // Feature 6 (unused)
  LIC_MUX_F7     = 0x07, // Feature 7 (unused)
  LIC_MUX_LDATE0 = 0x41,
  LIC_MUX_LDATE1 = 0x42,
  LIC_MUX_MAC    = 0x80,
  LIC_MUX_BDATE0 = 0x81,
  LIC_MUX_BDATE1 = 0x82,
  LIC_MUX_VIN0   = 0x83,
  LIC_MUX_VIN1   = 0x84,
  LIC_MUX_VIN2   = 0x85,
} LicenseMux;
typedef struct {
  uint8_t mux;
  uint8_t ready :1;
  uint8_t trial :1;
  uint8_t expired :1;
  uint8_t :1;
  uint8_t module :4;
  union {
    struct {
      uint8_t enabled :1;
      uint8_t trial :1;
      uint8_t :6;
      uint8_t :8;
      uint16_t trials_used;
      uint16_t trials_left;
    } license;
    struct {
        uint8_t ldate0;
        uint8_t ldate1;
        uint8_t ldate2;
        uint8_t ldate3;
        uint8_t ldate4;
        uint8_t ldate5;
    } ldate0;
    struct {
        uint8_t ldate6;
        uint8_t ldate7;
        uint8_t ldate8;
        uint8_t ldate9;
        uint8_t :8;
        uint8_t :8;
    } ldate1;    
    struct {
      uint8_t addr0;
      uint8_t addr1;
      uint8_t addr2;
      uint8_t addr3;
      uint8_t addr4;
      uint8_t addr5;
    } mac;
    struct {
      uint8_t date0;
      uint8_t date1;
      uint8_t date2;
      uint8_t date3;
      uint8_t date4;
      uint8_t date5;
    } bdate0;
    struct {
      uint8_t date6;
      uint8_t date7;
      uint8_t date8;
      uint8_t date9;
      uint8_t :8;
      uint8_t :8;
    } bdate1;
    struct {
      uint8_t vin00;
      uint8_t vin01;
      uint8_t vin02;
      uint8_t vin03;
      uint8_t vin04;
      uint8_t vin05;
    } vin0;
    struct {
      uint8_t vin06;
      uint8_t vin07;
      uint8_t vin08;
      uint8_t vin09;
      uint8_t vin10;
      uint8_t vin11;
    } vin1;
    struct {
      uint8_t vin12;
      uint8_t vin13;
      uint8_t vin14;
      uint8_t vin15;
      uint8_t vin16;
      uint8_t :8;
    } vin2;
  };
} MsgLicense;

typedef struct {
  uint8_t module;
  uint8_t platform;
  uint16_t major;
  uint16_t minor;
  uint16_t build;
} MsgVersion;

#define BUILD_ASSERT(cond) do { (void) sizeof(char [1 - 2*!(cond)]); } while(0)
static void dispatchAssertSizes() {
  BUILD_ASSERT(8 == sizeof(MsgBrakeCmd));
  BUILD_ASSERT(8 == sizeof(MsgBrakeReport));
  BUILD_ASSERT(8 == sizeof(MsgThrottleCmd));
  BUILD_ASSERT(8 == sizeof(MsgThrottleReport));
  BUILD_ASSERT(8 == sizeof(MsgSteeringCmd));
  BUILD_ASSERT(8 == sizeof(MsgSteeringReport));
  BUILD_ASSERT(1 == sizeof(MsgGearCmd));
  BUILD_ASSERT(2 == sizeof(MsgGearReport));
  BUILD_ASSERT(6 == sizeof(MsgReportAccel));
  BUILD_ASSERT(6 == sizeof(MsgReportGyro));
  BUILD_ASSERT(8 == sizeof(MsgLicense));
  BUILD_ASSERT(8 == sizeof(MsgVersion));
}
#undef BUILD_ASSERT

enum {
  ID_BRAKE_CMD              = 0x060,
  ID_BRAKE_REPORT           = 0x061,
  ID_THROTTLE_CMD           = 0x062,
  ID_THROTTLE_REPORT        = 0x063,
  ID_STEERING_CMD           = 0x064,
  ID_STEERING_REPORT        = 0x065,
  ID_GEAR_CMD               = 0x066,
  ID_GEAR_REPORT            = 0x067,
  ID_REPORT_ACCEL           = 0x06B,
  ID_REPORT_GYRO            = 0x06C,
  ID_LICENSE                = 0x07E,
  ID_VERSION                = 0x07F,
};

} // namespace dbw_polaris_can

#endif // _DBW_POLARIS_CAN_DISPATCH_H

