/*
** Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
** Markus Staeuble, Jonas Junger, Johannes Pankert, Philipp Leemann,
** Tom Lankhorst, Samuel Bachmann, Gabriel Hottiger, Lennert Nachtigall,
** Mario Mauerer, Remo Diethelm
**
** This file is part of the soem_interface.
**
** The soem_interface is free software: you can redistribute it and/or modify
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
** along with the soem_interface.  If not, see <https://www.gnu.org/licenses/>.
*/

//  soem_interface
#include <soem_interface/EthercatBusBase.hpp>
#include <soem_interface/EthercatSlaveBase.hpp>

namespace soem_interface {

EthercatSlaveBase::EthercatSlaveBase(EthercatBusBase* bus, const uint32_t address) : bus_(bus), address_(address) {}
EthercatSlaveBase::EthercatSlaveBase() : bus_(nullptr), address_(0) {}

// This definition must not be in the header, because of the forward declaration of EthercatBus
template <typename Value>
bool EthercatSlaveBase::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return bus_->sendSdoWrite(address_, index, subindex, completeAccess, value);
}

// This definition must not be in the header, because of the forward declaration of EthercatBus
template <typename Value>
bool EthercatSlaveBase::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return bus_->sendSdoRead(address_, index, subindex, completeAccess, value);
}

// These definitions must not be in the header, because of the forward declaration of EthercatBus
template bool EthercatSlaveBase::sendSdoWrite<int8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                      const int8_t value);
template bool EthercatSlaveBase::sendSdoWrite<int16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                       const int16_t value);
template bool EthercatSlaveBase::sendSdoWrite<int32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                       const int32_t value);
template bool EthercatSlaveBase::sendSdoWrite<int64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                       const int64_t value);
template bool EthercatSlaveBase::sendSdoWrite<uint8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                       const uint8_t value);
template bool EthercatSlaveBase::sendSdoWrite<uint16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                        const uint16_t value);
template bool EthercatSlaveBase::sendSdoWrite<uint32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                        const uint32_t value);
template bool EthercatSlaveBase::sendSdoWrite<uint64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                        const uint64_t value);
template bool EthercatSlaveBase::sendSdoWrite<float>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                     const float value);
template bool EthercatSlaveBase::sendSdoWrite<double>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                      const double value);
template bool EthercatSlaveBase::sendSdoWrite<std::string>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                           const std::string value);

template bool EthercatSlaveBase::sendSdoRead<int8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                     int8_t& value);
template bool EthercatSlaveBase::sendSdoRead<int16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                      int16_t& value);
template bool EthercatSlaveBase::sendSdoRead<int32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                      int32_t& value);
template bool EthercatSlaveBase::sendSdoRead<int64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                      int64_t& value);
template bool EthercatSlaveBase::sendSdoRead<uint8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                      uint8_t& value);
template bool EthercatSlaveBase::sendSdoRead<uint16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                       uint16_t& value);
template bool EthercatSlaveBase::sendSdoRead<uint32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                       uint32_t& value);
template bool EthercatSlaveBase::sendSdoRead<uint64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                       uint64_t& value);
template bool EthercatSlaveBase::sendSdoRead<float>(const uint16_t index, const uint8_t subindex, const bool completeAccess, float& value);
template bool EthercatSlaveBase::sendSdoRead<double>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                     double& value);
template bool EthercatSlaveBase::sendSdoRead<std::string>(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                                     std::string& value);

bool EthercatSlaveBase::sendSdoReadGeneric(const std::string& indexString, const std::string& subindexString,
                                           const std::string& valueTypeString, std::string& valueString) {
  printWarnNotImplemented();
  return false;
}

bool EthercatSlaveBase::sendSdoWriteGeneric(const std::string& indexString, const std::string& subindexString,
                                            const std::string& valueTypeString, const std::string& valueString) {
  printWarnNotImplemented();
  return false;
}

bool EthercatSlaveBase::sendSdoReadVisibleString(const uint16_t index, const uint8_t subindex, std::string& value) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return bus_->sendSdoReadVisibleString(address_, index, subindex, value);
}

}  // namespace soem_interface
