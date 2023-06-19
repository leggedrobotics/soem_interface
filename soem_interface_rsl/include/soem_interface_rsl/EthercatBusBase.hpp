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

#include <cassert>
// std
#include <atomic>
#include <chrono>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// soem_rsl

#include <message_logger/message_logger.hpp>
// soem_interface_rsl

#include <soem_interface_rsl/common/soem_rsl_export.h>
#include <soem_interface_rsl/common/EthercatTypes.hpp>
#include <soem_interface_rsl/common/ExtendedRegisters.hpp>
#include <soem_interface_rsl/common/Macros.hpp>
#include <soem_interface_rsl/common/ObjectDictionaryUtilities.hpp>
#include <soem_interface_rsl/common/ThreadSleep.hpp>

namespace soem_interface_rsl {

// forward declaration for EthercatSlaveBase
class EthercatSlaveBase;
using EthercatSlaveBasePtr = std::shared_ptr<EthercatSlaveBase>;

// Template Adatper for pImpl Design with Templates. https://en.cppreference.com/w/cpp/language/pimpl
class SOEM_RSL_EXPORT EthercatBusBaseTemplateAdapter {
 private:
  struct EthercatSlaveBaseImpl;

 protected:
  std::unique_ptr<EthercatSlaveBaseImpl> pImpl_;
  bool sdoWriteForward(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, int size, void* buf);
  bool sdoReadForward(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, int size, void* buf);
  int sdoReadSizeForward(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, int size,
                         void* buf);
  void readTxPdoForward(const uint16_t slave, int size, void* buf) const;
  void writeRxPdoForward(const uint16_t slave, int size, const void* buf);

 public:
  explicit EthercatBusBaseTemplateAdapter(const std::string& name);
  ~EthercatBusBaseTemplateAdapter();
};

/**
 * @brief      Class for managing an ethercat bus containing one or multpile
 *             slaves
 */
class SOEM_RSL_EXPORT EthercatBusBase : private EthercatBusBaseTemplateAdapter {
 public:
  using PdoSizePair = std::pair<uint16_t, uint16_t>;
  using PdoSizeMap = std::unordered_map<std::string, PdoSizePair>;

  EthercatBusBase() = delete;
  /*!
   * Constructor.
   * @param name Name of the bus, e.g. "eth0".
   */
  explicit EthercatBusBase(const std::string& name);

  /*!
   * Destructor.
   */
  virtual ~EthercatBusBase();

  /*!
   * Get the name of the bus.
   * @return Name of the bus.
   */
  const std::string& getName() const;

  /*!
   * Check if a bus is available.
   * @param name Name of the bus.
   * @return True if available.
   */
  static bool busIsAvailable(const std::string& name);

  /*!
   * Print all available busses.
   */
  static void printAvailableBusses();

  /*!
   * Check if this bus is available.
   * @return True if available.
   */
  bool busIsAvailable() const;

  /*!
   * Get the number of slaves which were detected on this bus.
   * @return Number of slaves.
   */
  int getNumberOfSlaves() const;

  /*!
   * Add an EtherCAT slave.
   * @slave EtherCAT slave.
   * @return True if successful.
   */
  bool addSlave(const EthercatSlaveBasePtr& slave);

  /*!
   * Startup the bus communication.
   * @param abortFlag  during startup it is waited till all the slaves are ready this can take some time, the abortFlag can be set to abort
   * this operation.
   * @param sizeCheck	perform a check of the Rx and Tx Pdo sizes defined in the PdoInfo oject of the slaves
   * @param maxDiscoverRetries	number of retries till the configured number of slaves are found on the bus.
   * @return True if successful.
   */
  bool startup(bool sizeCheck, int maxDiscoverRetries = 10);

  bool startup(std::atomic<bool>& abortFlag, bool sizeCheck, int maxDiscoverRetries = 10);

  /*!
   * Update step 1: Read all PDOs.
   */
  void updateRead();

  /*!
   * Update step 2: Write all PDOs.
   */
  void updateWrite();

  /*!
   * Get the time of the last successful PDO reading, not threadsafe.
   * @return Stamp.
   */
  const std::chrono::time_point<std::chrono::high_resolution_clock>& getUpdateReadStamp() const;

  /*!
   * Get the time of the last successful PDO writing, not threadsafe
   * @return Stamp.
   */
  const std::chrono::time_point<std::chrono::high_resolution_clock>& getUpdateWriteStamp() const;

  /*!
   * Shutdown the bus communication.
   */
  void shutdown();

  /*!
   * Set the desired EtherCAT state machine state.
   * @param state Desired state.
   * @param slave Address of the slave, default  0 for all slaves.
   */
  void setState(const uint16_t state, const uint16_t slave = 0);

  /*!
   * Set the desired EtherCAT state machine state.
   * @param state Desired state for all slaves on the bus.
   * @param slave Address of the slave, default  0 for all slaves.
   */
  void setState(ETHERCAT_SM_STATE state, const uint16_t slave = 0);

  /*!
   * Wait for an EtherCAT state machine state to be reached.
   * @param state      Desired state.
   * @param slave      Address of the slave, 0 for all slaves.
   * @param maxRetries Maximum number of retries.
   * @param retrySleep Duration to sleep between the retries.
   * @return True if the state has been reached within the timeout.
   */
  bool waitForState(const uint16_t state, const uint16_t slave = 0, const unsigned int maxRetries = 40);
  bool waitForState(ETHERCAT_SM_STATE state, const uint16_t slave = 0, const unsigned int maxRetries = 40);

  /*!
   * Reads the ethercat state machine state, updates the state information of all slaves.
   * Therefore reads all slaves in case not all slaves are in the same state.
   * @param slave address of the slave, 0 for the lowest state of all slaves
   * @return Ethercat State Machine Enum.
   */

  soem_interface_rsl::ETHERCAT_SM_STATE getEthercatState(const uint16_t slave = 0);

  /*!
   * Checks if all slaves are in EC_STATE_OPERATIONAL, therefore reads EC state from all slaves!
   * If not does some basic printing for potential debugging.
   * @param logErrorCounterForDiagnosis runs some error counter diagnosis, which are available to log in getBusDiagnosis.
   * @return true if all fine = all slaves in EC_STATE_OP
   */
  bool doBusMonitoring(bool logErrorCounterForDiagnosis = false);

  /*!
   * @param if returns true busDiagnosisLogOut gets updated with the newest data.
   * @return Return true if the busDiagnosisLog got updated.
   * Note: needs to be called within the same thread as doBusMonitoring, not threadsafe!!
   */

  bool getBusDiagnosisLog(BusDiagnosisLog& busDiagnosisLogOut);

  /*!
   * Generate and return the state string.
   * @param state EtherCAT.
   * @return The state string for logging.
   */
  static std::string getStateString(uint16_t state);

  /*!
   * Synchronize the distributed clocks.
   *
   * @param      slave     Address of the slave.
   * @param      activate  True to activate the distr. clock, false to
   *                       deactivate.
   * @param[in]  timeStep  The time step
   */
  void syncDistributedClock0(const uint16_t slave, const bool activate, const double cycleTime, const double cycleShift);

  /*!
   * Returns a map of the actually requested PDO sizes (Rx & Tx) This is useful
   * for slaves where the PDO size at startup is unknown This method shall be
   * used after adding the slaves and after executing the "startup" method
   *
   * @return     std::unordered_map with the addresses and the corresponding Pdo
   *             sizes
   */
  PdoSizeMap getHardwarePdoSizes();
  PdoSizePair getHardwarePdoSizes(const uint16_t slave);

  /*!
   * Send a writing SDO.
   * @param slave          Address of the slave.
   * @param index          Index of the SDO.
   * @param subindex       Sub-index of the SDO.
   * @param completeAccess Access all sub-indices at once.
   * @param value          Value to write.
   * @return True if successful.
   */
  template <typename Value>
  bool sendSdoWrite(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value) {
    const int size = sizeof(Value);
    Value valueCopy = value;  // copy value to make it modifiable
    return sdoWriteForward(slave, index, subindex, completeAccess, size, &valueCopy);
  }

  /*!
   * Send a reading SDO.
   * @param slave          Address of the slave.
   * @param index          Index of the SDO.
   * @param subindex       Sub-index of the SDO.
   * @param completeAccess Access all sub-indices at once.
   * @param value          Return argument, will contain the value which was read.
   * @return True if successful.
   */
  template <typename Value>
  bool sendSdoRead(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value) {
    int size = sizeof(Value);
    return sdoReadForward(slave, index, subindex, completeAccess, size, &value);
  }

  /**
   * Send a special reading SDO to read SDOs of type visible string.
   * @param slave          Address of the slave.
   * @param index          Index of the SDO.
   * @param subindex       Sub-index of the SDO.
   * @param value          Return argument, will contain the value which was read.
   * @return True if successful.
   */
  bool sendSdoReadVisibleString(const uint16_t slave, const uint16_t index, const uint8_t subindex, std::string& value);

  /*!
   * Check if the bus is ok.
   * @return True if bus is ok.
   */
  [[nodiscard]] bool busIsOk() const;

  /*!
   * Read a TxPDO from the buffer.
   * @param slave Address of the slave.
   * @param txPdo Return argument, TxPDO container.
   */
  template <typename TxPdo>
  void readTxPdo(const uint16_t slave, TxPdo& txPdo) const {
    std::byte buffer[sizeof(TxPdo)];
    readTxPdoForward(slave, sizeof(TxPdo), buffer);
    memcpy(&txPdo, buffer, sizeof(TxPdo));
  }

  /*!
   * Write an RxPDO to the buffer.
   * @param slave Address of the slave.
   * @param rxPdo RxPDO container.
   */
  template <typename RxPdo>
  void writeRxPdo(const uint16_t slave, const RxPdo& rxPdo) {
    std::byte buffer[sizeof(RxPdo)];
    memcpy(buffer, &rxPdo, sizeof(RxPdo));
    writeRxPdoForward(slave, sizeof(RxPdo), buffer);
  }
};

using EthercatBusBasePtr = std::shared_ptr<EthercatBusBase>;

/*!
 * Send a reading SDO - specialization for strings
 * @param slave          Address of the slave.
 * @param index          Index of the SDO.
 * @param subindex       Sub-index of the SDO.
 * @param completeAccess Access all sub-indices at once.
 * @param value          Return argument, will contain the value which was read. The string needs to be preallocated to the correct size!
 * @return True if successful.
 */
template <>
bool EthercatBusBase::sendSdoRead<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                               const bool completeAccess, std::string& value);

/*!
 * Send a writing SDO - specialization for strings
 * @param slave          Address of the slave.
 * @param index          Index of the SDO.
 * @param subindex       Sub-index of the SDO.
 * @param completeAccess Access all sub-indices at once.
 * @param value          Value to write.
 * @return True if successful.
 */
template <>
bool EthercatBusBase::sendSdoWrite<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                                const bool completeAccess, const std::string value);
}  // namespace soem_interface_rsl
