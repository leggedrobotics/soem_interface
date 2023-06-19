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

#ifndef ETHERCAT_WS_EXTENDEDREGISTERS_HPP
#define ETHERCAT_WS_EXTENDEDREGISTERS_HPP

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

namespace soem_interface_rsl {

enum RegTypeEnum : uint16_t { Unsigned1 = 1, Unsigned8 = 1, Unsigned16 = 2, Unsigned32 = 4, Unsigned64 = 8 };

template <class Enum_>
struct RegEntry {
  using RegAddrEnum = Enum_;
  const uint16_t addr{};
  RegAddrEnum addrEnum;
  const RegTypeEnum regTypeEnum{};
  const std::string_view name{};
  constexpr RegEntry(Enum_ addr_, RegTypeEnum regTypeEnum_, std::string_view name_)
      : addr(static_cast<uint16_t>(addr_)), addrEnum(addr_), regTypeEnum(regTypeEnum_), name(name_){};
};

template <class RegEntryType>
struct RegisterDefinition {
  using RegAddrEnum = typename RegEntryType::RegAddrEnum;
  static_assert(std::is_same_v<std::underlying_type_t<RegAddrEnum>, uint16_t>);
  std::array<RegEntry<RegAddrEnum>, static_cast<size_t>(RegAddrEnum::SIZE)> Registers;

  [[nodiscard]] constexpr uint16_t memorySize() const {
    uint16_t size{0};
    for (const auto& reg : Registers) {
      size += reg.regTypeEnum;
    }
    return size;
  }
  using iterator = typename std::array<RegEntryType, static_cast<size_t>(RegAddrEnum::SIZE)>::iterator;
  using const_iterator = typename std::array<RegEntryType, static_cast<size_t>(RegAddrEnum::SIZE)>::const_iterator;
  [[nodiscard]] constexpr const_iterator begin() const { return Registers.begin(); }
  [[nodiscard]] constexpr const_iterator end() const { return Registers.end(); }
  [[nodiscard]] constexpr const_iterator cbegin() const { return Registers.cbegin(); }
  [[nodiscard]] constexpr const_iterator cend() const { return Registers.cend(); }

  [[nodiscard]] constexpr RegTypeEnum regTypeEnumByAddr(RegAddrEnum regAddrEnum) const {
    for (const auto& reg : Registers) {
      if (reg.addr == static_cast<uint16_t>(regAddrEnum)) {
        return reg.regTypeEnum;
      }
    }
    throw std::runtime_error("Could not find Register by regAddrsEnum");
  };

  [[nodiscard]] constexpr uint16_t offsetByAddr(RegAddrEnum regAddrEnum) const {
    uint16_t offset{0};
    for (const auto& reg : Registers) {
      if (reg.addr == static_cast<uint16_t>(regAddrEnum)) {
        return offset;
      }
      offset += reg.regTypeEnum;
    }
    throw std::runtime_error("Could not find Register by regAddrsEnum");
  };

  // calcualtes the size required to read up to/including the given register.
  [[nodiscard]] constexpr uint16_t sizeUpToAddr(RegAddrEnum regAddrEnum) const {
    uint16_t size{0};
    for (const auto& reg : Registers) {
      size += reg.regTypeEnum;
      if (reg.addr == static_cast<uint16_t>(regAddrEnum)) {
        return size;
      }
    }
    throw std::runtime_error("Could not calc size by endAddr");
  }

  template <class ReturnType>
  constexpr auto getValueFromRawAs(RegAddrEnum regAddrEnum, void* rawData, [[maybe_unused]] const uint16_t rawSize) const {
    [[maybe_unused]] const uint16_t offset = offsetByAddr(regAddrEnum);
    [[maybe_unused]] const uint16_t valueSize = regTypeEnumByAddr(regAddrEnum);  // assertion only in debug.
    assert(offset + valueSize - 1 < rawSize);                                    // todo should be static assert.
    assert(sizeof(ReturnType) == valueSize);                                     // todo should be static assert.

    ReturnType rawValue{};
    std::memcpy(&rawValue, (static_cast<char*>(rawData) + offset), sizeof(ReturnType));

    //handle endianess. (default little endinaness)  ethos = ethercat-to-host-short
//    switch (regTypeEnumByAddr(regAddrEnum)) {
//      case RegTypeEnum::Unsigned8:
//        break;
//      case RegTypeEnum::Unsigned16:
//        rawValue = etohs(rawValue);
//        break;
//      case RegTypeEnum::Unsigned32:
//        rawValue = etohl(rawValue);
//        break;
//      case RegTypeEnum::Unsigned64:
//        rawValue = etohll(rawValue);
//        break;
//      default:
//        throw std::runtime_error("check endianess when converting larger memory blobs..");
//    }

    return rawValue;
  }
};

struct SOEM_RSL_EXPORT REG  {
  enum class ERROR_COUNTERS : uint16_t {
    FRAME_ERROR_PORT0_ADDR = 0x0300,
    PHYSICAL_ERROR_PORT0_ADDR = 0x0301,
    FRAME_ERROR_PORT1_ADDR = 0x0302,
    PHYSICAL_ERROR_PORT1_ADDR = 0x0303,
    FRAME_ERROR_PORT2_ADDR = 0x0304,
    PHYSICAL_ERROR_PORT2_ADDR = 0x0305,
    FRAME_ERROR_PORT3_ADDR = 0x0306,
    PHYSICAL_ERROR_PORT3_ADDR = 0x0307,
    PREVIOUS_ERROR_CNT_PORT0 = 0x0308,
    PREVIOUS_ERROR_CNT_PORT1 = 0x0309,
    PREVIOUS_ERROR_CNT_PORT2 = 0x030A,
    PREVIOUS_ERROR_CNT_PORT3 = 0x030B,
    MALFORMAT_FRAME_CNT = 0x030C,
    LOCAL_PROBLEM_CNT = 0x030D,
    LOST_LINK_CNT_PORT0 = 0x0310,
    LOST_LINK_CNT_PORT1 = 0x0311,
    LOST_LINK_CNT_PORT2 = 0x0312,
    LOST_LINK_CNT_PORT3 = 0x0313,
    SIZE = 18  // 18 bytes - we can easily read this in one datagram per slave.
  };

  static constexpr RegisterDefinition<RegEntry<ERROR_COUNTERS>> ERROR_COUNTERS_LIST{
      RegEntry(ERROR_COUNTERS::FRAME_ERROR_PORT0_ADDR, RegTypeEnum::Unsigned8, "Frame error count port 0"),
      RegEntry(ERROR_COUNTERS::PHYSICAL_ERROR_PORT0_ADDR, RegTypeEnum::Unsigned8, "Physical error count port 0"),
      RegEntry(ERROR_COUNTERS::FRAME_ERROR_PORT1_ADDR, RegTypeEnum::Unsigned8, "Frame error count port 1"),
      RegEntry(ERROR_COUNTERS::PHYSICAL_ERROR_PORT1_ADDR, RegTypeEnum::Unsigned8, "Physical error count port 1"),
      RegEntry(ERROR_COUNTERS::FRAME_ERROR_PORT2_ADDR, RegTypeEnum::Unsigned8, "Frame error count prot 2"),
      RegEntry(ERROR_COUNTERS::PHYSICAL_ERROR_PORT2_ADDR, RegTypeEnum::Unsigned8, "Physical error count port 2"),
      RegEntry(ERROR_COUNTERS::FRAME_ERROR_PORT3_ADDR, RegTypeEnum::Unsigned8, "Frame error count port 3"),
      RegEntry(ERROR_COUNTERS::PHYSICAL_ERROR_PORT3_ADDR, RegTypeEnum::Unsigned8, "Physical error count port 3"),
      RegEntry(ERROR_COUNTERS::PREVIOUS_ERROR_CNT_PORT0, RegTypeEnum::Unsigned8, "Port 0 previous error count"),
      RegEntry(ERROR_COUNTERS::PREVIOUS_ERROR_CNT_PORT1, RegTypeEnum::Unsigned8, "Port 1 previous error count"),
      RegEntry(ERROR_COUNTERS::PREVIOUS_ERROR_CNT_PORT2, RegTypeEnum::Unsigned8, "Port 2 previous error count"),
      RegEntry(ERROR_COUNTERS::PREVIOUS_ERROR_CNT_PORT3, RegTypeEnum::Unsigned8, "Port 3 previous error count"),
      RegEntry(ERROR_COUNTERS::MALFORMAT_FRAME_CNT, RegTypeEnum::Unsigned8, "Malformat frame counter"),
      RegEntry(ERROR_COUNTERS::LOCAL_PROBLEM_CNT, RegTypeEnum::Unsigned8, "Local problem counter"),
      RegEntry(ERROR_COUNTERS::LOST_LINK_CNT_PORT0, RegTypeEnum::Unsigned8, "Lost link count port 0"),
      RegEntry(ERROR_COUNTERS::LOST_LINK_CNT_PORT1, RegTypeEnum::Unsigned8, "Lost link count port 1"),
      RegEntry(ERROR_COUNTERS::LOST_LINK_CNT_PORT2, RegTypeEnum::Unsigned8, "Lost link count port 2"),
      RegEntry(ERROR_COUNTERS::LOST_LINK_CNT_PORT3, RegTypeEnum::Unsigned8, "Lost link count port 3")};

  struct Counter {
    uint8_t previousValue{0};
    unsigned long fullValue{0};

    friend std::ostream& operator<<(std::ostream& os, Counter const& p) { return os << p.fullValue; }
  };

  //
  //  //read as pack and check with DLStatus. --> already done by soem_rsl.
  //  enum class DL_STATUS : uint16_t{
  //    DL_STATUS_ADDR = 0x0110,
  //    SIZE = 1
  //  };
  //  static constexpr RegisterDefinition<RegEntry<DL_STATUS>> DL_STATUS_LIST{
  //      RegEntry(DL_STATUS::DL_STATUS_ADDR, RegTypeEnum::Unsigned16, "DL status")
  //  };
  //
  //  PACKED_BEGIN
  //  struct PACKED DLStatus{
  //    unsigned PdiOperational: 1;
  //    unsigned DLSuserWatchdogStatus: 1;
  //    unsigned ExtendedLinkDetection: 1;
  //    unsigned Reserved1: 1;
  //    unsigned LinkStatusPort0: 1;
  //    unsigned LinkStatusPort1: 1;
  //    unsigned LinkStatusPort2: 1;
  //    unsigned LinkStatusPort3: 1;
  //    unsigned LoopStatusPort0: 1;
  //    unsigned SignalDetectionPort0: 1;
  //    unsigned LoopStatusPort1: 1;
  //    unsigned SignalDetectionPort1: 1;
  //    unsigned LoopStatusPort2: 1;
  //    unsigned SignalDetectionPort2: 1;
  //    unsigned LoopStatusPort3: 1;
  //    unsigned SignalDetectionPort3: 1;
  //  };
  //  PACKED_END
};

struct BusDiagnosisLog {
  bool fullyUpdated{false};
  std::vector<std::array<REG::Counter, static_cast<uint16_t>(REG::ERROR_COUNTERS::SIZE)>> errorCounters_{};
  // the OR of all slaves ALStatusCode.
  uint16_t ecatApplicationLayerStatus{};
};

}  // namespace soem_interface_rsl

#endif  // ETHERCAT_WS_EXTENDEDREGISTERS_HPP
