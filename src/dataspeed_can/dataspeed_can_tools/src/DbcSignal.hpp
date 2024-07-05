/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2020, Dataspeed Inc.
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

#ifndef _DBC_SIGNAL_HPP
#define _DBC_SIGNAL_HPP

#include <string>
#include <iosfwd>
#include <set>

typedef enum {
  MOTOROLA,
  INTEL
} ByteOrder;

typedef enum {
  UNSIGNED,
  SIGNED
} Sign;

typedef enum {
  NONE,
  MULTIPLEXED,
  MULTIPLEXOR
} Multiplexor;


/**
 * This class represents a Signal contained in a Message of a DBC-File.
 * One can Query all the necessary information from this class to define
 * a Signal
 */
class Signal {

  typedef std::set<std::string> toList;
  // The name of the Signal in the DBC-File
  std::string name;
  // The Byteorder of the Signal (@see: endianess)
  ByteOrder order;
  // The Startbit inside the Message of this Signal. Allowed values are 0-63
  unsigned short startBit;
  // The Length of the Signal. It can be anything between 1 and 64
  unsigned short length;
  // If the Data contained in the Signal is signed or unsigned Data
  Sign sign;
  // Depending on the information given above one can calculate the minimum of this Signal
  double minimum;
  // Depending on the information given above one can calculate the maximum of this Signal
  double maximum;
  // The Factor for calculating the physical value: phys = digits * factor + offset
  double factor;
  // The offset for calculating the physical value: phys = digits * factor + offset
  double offset;
  // String containing an associated unit.
  std::string unit;
  // Contains weather the Signal is Multiplexed and if it is, multiplexNum contains multiplex number
  Multiplexor multiplexor;
  // Contains the multiplex Number if the Signal is multiplexed
  unsigned short multiplexNum;
  // Contains to which Control Units in the CAN-Network the Signal shall be sent
  toList to;

public:
  // Overload of operator>> to allow parsing from DBC Streams
  friend std::istream& operator>>(std::istream& in, Signal& sig);

  // Getter for all the Values contained in a Signal
  const std::string& getName() const { return name; }
  ByteOrder getByteOrder() const { return order; }
  unsigned short getStartbit() const { return startBit; }
  unsigned short getLength() const { return length; }
  Sign getSign() const { return sign; }
  double getMinimum() const { return minimum; }
  double getMaximum() const { return maximum; }
  double getFactor() const { return factor; }
  double getOffset() const { return offset; }
  const std::string& getUnit() const { return unit; }
  Multiplexor getMultiplexor() const { return multiplexor; }
  unsigned short getMultiplexedNumber() const { return multiplexNum; }
  const toList& getTo() const { return to; }

};

#endif /* _DBC_SIGNAL_HPP */

