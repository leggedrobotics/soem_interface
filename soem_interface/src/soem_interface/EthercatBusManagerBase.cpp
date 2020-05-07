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

//  anydrive
#include "soem_interface/EthercatBusManagerBase.hpp"

namespace soem_interface {

bool EthercatBusManagerBase::addEthercatBus(soem_interface::EthercatBusBase* bus) {
  if (bus == nullptr) {
    MELO_ERROR_STREAM("[RokubiminiEthercatBusManager::addEthercatBus] bus is nullptr")
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  const auto& it = buses_.find(bus->getName());
  if (it == buses_.end()) {
    buses_.insert(std::make_pair(bus->getName(), std::unique_ptr<soem_interface::EthercatBusBase>(bus)));
    return true;
  } else {
    return false;
  }
}

bool EthercatBusManagerBase::addEthercatBus(std::unique_ptr<soem_interface::EthercatBusBase> bus) {
  if (bus == nullptr) {
    MELO_ERROR_STREAM("[RokubiminiEthercatBusManager::addEthercatBus] bus is nullptr")
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  const auto& it = buses_.find(bus->getName());
  if (it == buses_.end()) {
    buses_.insert(std::make_pair(bus->getName(), std::move(bus)));
    return true;
  } else {
    return false;
  }
}

bool EthercatBusManagerBase::startupAllBuses() {
  bool success = startupCommunication();
  setBussesOperational();
  return success;
}

void EthercatBusManagerBase::setBussesOperational() {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  // Only set the state but do not wait for it, since some devices (e.g. junctions) might not be able to reach it.
  for (auto& bus : buses_) {
    bus.second->setState(EC_STATE_OPERATIONAL);
  }
}

void EthercatBusManagerBase::setBussesPreOperational() {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  // Only set the state but do not wait for it, since some devices (e.g. junctions) might not be able to reach it.
  for (auto& bus : buses_) {
    bus.second->setState(EC_STATE_PRE_OP);
  }
}

void EthercatBusManagerBase::setBussesSafeOperational() {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  // Only set the state but do not wait for it, since some devices (e.g. junctions) might not be able to reach it.
  for (auto& bus : buses_) {
    bus.second->setState(EC_STATE_SAFE_OP);
  }
}

void EthercatBusManagerBase::waitForState(
  const uint16_t state,
  const uint16_t slave,
  const std::string busName,
  const unsigned int maxRetries,
  const double retrySleep) {

  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  if(busName.empty()) {
    for (auto& bus : buses_) {
      bus.second->waitForState(state, slave, maxRetries, retrySleep);
    }
  }
  else {
    buses_.at(busName)->waitForState(state, slave, maxRetries, retrySleep);
  }

}

bool EthercatBusManagerBase::startupCommunication() {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_) {
    if (!bus.second->startup()) {
      MELO_ERROR_STREAM("Failed to startup bus '" << bus.first << "'.");
      return false;
    }
  }
  return true;
}

void EthercatBusManagerBase::readAllBuses() {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_) {
    bus.second->updateRead();
  }
}

void EthercatBusManagerBase::writeToAllBuses() {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_) {
    bus.second->updateWrite();
  }
}

void EthercatBusManagerBase::shutdownAllBuses() {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_) {
    bus.second->shutdown();
  }
}

std::unique_ptr<EthercatBusBase> EthercatBusManagerBase::extractBusByName(const std::string& name) {
  std::unique_ptr<EthercatBusBase> busOut = std::move(buses_.at(name));
  buses_.erase(name);
  return busOut;
}

EthercatBusManagerBase::BusMap EthercatBusManagerBase::extractBuses() {
  BusMap busMapOut;

  for(auto& bus : buses_) {
    busMapOut.insert(std::make_pair(bus.first, std::move(bus.second)));
  }

  buses_.clear();
  return busMapOut;
}

bool EthercatBusManagerBase::allBusesAreOk() {
  for (const auto& bus : buses_) {
    if (!bus.second->busIsOk()) {
      return false;
    }
  }
  return true;
}

}  // namespace soem_interface
