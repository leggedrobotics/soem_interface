/*
** Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
** Markus Staeuble, Jonas Junger, Johannes Pankert, Philipp Leemann,
** Tom Lankhorst, Samuel Bachmann, Gabriel Hottiger, Lennert Nachtigall,
** Mario Mauerer, Remo Diethelm
**
** This file is part of the soem_interface_rsl.
**
** The soem_interface_rsl is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** The seom_interface is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with the soem_interface_rsl.  If not, see <https://www.gnu.org/licenses/>.
*/
#pragma once

#include <cstddef>
#include <cstdint>
#include "soem_rsl_export.h"

namespace soem_interface_rsl {

// mainly reexports some of SOEM's Macros in a namespace fashion to not collide with any other soem installations on the system.

// namespaced export of the EC_STATE from soem for ethercat slave sdks.
enum class SOEM_RSL_EXPORT ETHERCAT_SM_STATE : uint16_t {
  /** No valid state. */
  NONE = 0x00,
  /** Init state*/
  INIT = 0x01,
  /** Pre-operational. */
  PRE_OP = 0x02,
  /** Boot state*/
  BOOT = 0x03,
  /** Safe-operational. */
  SAFE_OP = 0x04,
  /** Operational */
  OPERATIONAL = 0x08,
  /** Error or ACK error */
  ACK = 0x10,
  ERROR = 0x10
};

enum class SOEM_RSL_EXPORT ETHERCAT_TYPE : uint16_t {
  BOOLEAN = 0x0001,
  INTEGER8 = 0x0002,
  INTEGER16 = 0x0003,
  INTEGER32 = 0x0004,
  UNSIGNED8 = 0x0005,
  UNSIGNED16 = 0x0006,
  UNSIGNED32 = 0x0007,
  REAL32 = 0x0008,
  VISIBLE_STRING = 0x0009,
  OCTET_STRING = 0x000A,
  UNICODE_STRING = 0x000B,
  TIME_OF_DAY = 0x000C,
  TIME_DIFFERENCE = 0x000D,
  DOMAIN = 0x000F,
  INTEGER24 = 0x0010,
  REAL64 = 0x0011,
  INTEGER64 = 0x0015,
  UNSIGNED24 = 0x0016,
  UNSIGNED64 = 0x001B,
  BIT1 = 0x0030,
  BIT2 = 0x0031,
  BIT3 = 0x0032,
  BIT4 = 0x0033,
  BIT5 = 0x0034,
  BIT6 = 0x0035,
  BIT7 = 0x0036,
  BIT8 = 0x0037
};

// note this is a simplification - the ethercat specs have a more fine grained access control. (Ecat specificiation Part 6, page 61.)
namespace EcAccess {  // make sure that no name conflicts, but allowing implicit conversion.
enum EcAccess : uint16_t {
  NOT_IMPL = 0b0000'0000'0000'0000,
  WO = 0b0000'0000'0011'1000,
  RO = 0b0000'0000'0000'0111,
  RW = 0b0000'0000'0011'1111
  // more specialization.
};
}

}  // namespace soem_interface_rsl
