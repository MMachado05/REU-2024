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

#ifndef _DBW_POLARIS_CAN_PEDAL_LUT_H
#define _DBW_POLARIS_CAN_PEDAL_LUT_H
#include <math.h>

namespace dbw_polaris_can
{

static const struct {float pedal; float percent;} THROTTLE_TABLE[] = {
// Duty,   %
 {0.200, 0.000},
 {0.300, 0.001},
 {0.800, 1.000},
};
static inline float throttlePedalFromPercent(float percent) {
  const unsigned int size = sizeof(THROTTLE_TABLE) / sizeof(THROTTLE_TABLE[0]);
  if (percent <= THROTTLE_TABLE[0].percent) {
    return THROTTLE_TABLE[0].pedal;
  } else if (percent >= THROTTLE_TABLE[size - 1].percent) {
    return THROTTLE_TABLE[size - 1].pedal;
  } else {
    for (unsigned int i = 1; i < size; i++) {
      if (percent < THROTTLE_TABLE[i].percent) {
        float start = THROTTLE_TABLE[i - 1].pedal;
        float dinput = percent - THROTTLE_TABLE[i - 1].percent;
        float dpedal = THROTTLE_TABLE[i].pedal - THROTTLE_TABLE[i - 1].pedal;
        float dpercent = THROTTLE_TABLE[i].percent - THROTTLE_TABLE[i - 1].percent;
        if (fabsf(dpercent) > (float)1e-6) {
          return start + (dinput * dpedal / dpercent);
        } else {
          return start + (dpedal / 2);
        }
      }
    }
  }
  return 0.0;
}

static inline float throttlePercentFromPedal(float pedal) {
  const unsigned int size = sizeof(THROTTLE_TABLE) / sizeof(THROTTLE_TABLE[0]);
  if (pedal <= THROTTLE_TABLE[0].pedal) {
    return THROTTLE_TABLE[0].percent;
  } else if (pedal >= THROTTLE_TABLE[size - 1].pedal) {
    return THROTTLE_TABLE[size - 1].percent;
  } else {
    for (unsigned int i = 1; i < size; i++) {
      if (pedal < THROTTLE_TABLE[i].pedal) {
        float start = THROTTLE_TABLE[i - 1].percent;
        float dinput = pedal - THROTTLE_TABLE[i - 1].pedal;
        float dpercent = THROTTLE_TABLE[i].percent - THROTTLE_TABLE[i - 1].percent;
        float dpedal = THROTTLE_TABLE[i].pedal - THROTTLE_TABLE[i - 1].pedal;
        if (fabsf(dpedal) > (float) 1e-6) {
          return start + (dinput * dpercent / dpedal);
        } else {
          return start + (dpercent / 2);
        }
      }
    }
  }
  return 0.0;
}

} // namespace dbw_polaris_can

#endif // _DBW_POLARIS_CAN_PEDAL_LUT_H

