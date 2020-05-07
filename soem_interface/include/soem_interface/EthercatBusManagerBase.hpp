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

#pragma once

// std
#include <map>
#include <memory>
#include <mutex>

#include <soem_interface/EthercatBusBase.hpp>

namespace soem_interface {

/**
 * @brief      Class for managing multiple ethercat busses
 */
class EthercatBusManagerBase {
 public:
  using BusMap = std::unordered_map<std::string, std::unique_ptr<EthercatBusBase>>;

  EthercatBusManagerBase() = default;
  virtual ~EthercatBusManagerBase() = default;

  /**
   * @brief      Adds an ethercat bus to the manager. The manager takes
   *             ownership of the bus pointer
   *
   * @param      bus   Raw bus ptr
   *
   * @return     True if bus has been added successfully
   */
  bool addEthercatBus(soem_interface::EthercatBusBase* bus);

  /**
   * @brief      Adds an ethercat bus to the manager. The manager takes
   *             ownership of the bus pointer
   *
   * @param      bus   Unique bus ptr
   *
   * @return     True if bus has been added successfully
   */
  bool addEthercatBus(std::unique_ptr<soem_interface::EthercatBusBase> bus);

  /**
   * @brief      Starts up all busses and puts them in operational mode
   *
   * @return     True if successful
   */
  bool startupAllBuses();

  /**
   * @brief      Starts up all busses
   *
   * @return     True if successful
   */
  bool startupCommunication();

  /**
   * @brief      Sets all busses to safe operational state
   */
  void setBussesSafeOperational();

  /**
   * @brief      Sets all busses to pre operational state
   */
  void setBussesPreOperational();

  /**
   * @brief      Sets all busses to operational state
   */
  void setBussesOperational();

  /**
   * @brief      Waits for the slave to reach a state
   *
   * @param[in]  state       Ethercat state
   * @param[in]  slave       Slave address, 0 = all slaves
   * @param[in]  busName     The name of the bus
   * @param[in]  maxRetries  The maximum retries
   * @param[in]  retrySleep  The retry sleep
   */
  void waitForState(
    const uint16_t state,
    const uint16_t slave = 0,
    const std::string busName = "",
    const unsigned int maxRetries = 40,
    const double retrySleep = 0.001);

  /**
   * @brief      Calls update read on all busses
   */
  void readAllBuses();

  /**
   * @brief      Calls update write on all busses
   */
  void writeToAllBuses();

  /**
   * @brief      Calls shutdown on all busses
   */
  void shutdownAllBuses();

  /**
   * @brief      Returns a non owning bus pointer. The manager still manages all
   *             the buses
   *
   * @param[in]  name  The bus name
   *
   * @return     A shared_ptr to the bus.
   */
  EthercatBusBase* getBusByName(const std::string& name) const { return buses_.at(name).get(); }

  /**
   * @brief      Returns an owning bus pointer. The manager is now not managing
   *             this bus anymore
   *
   * @param[in]  name  The bus name
   *
   * @return     A unique_ptr to the bus.
   */
  std::unique_ptr<EthercatBusBase> extractBusByName(const std::string& name);

  /**
   * @brief      Extracts all buses from the manager. The manager is now not
   *             managing any buses anymore
   *
   * @return     Bus map
   */
  BusMap extractBuses();

  /**
   * Check if all buses are ok.
   * @return True if all buses are ok.
   */
  bool allBusesAreOk();

 protected:
  // Mutex prohibiting simultaneous access to EtherCAT bus manager.
  std::recursive_mutex busMutex_;
  BusMap buses_;
};

using EthercatBusManagerBasePtr = std::shared_ptr<EthercatBusManagerBase>;

}  // namespace soem_interface
