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

#include <soem_interface_rsl/EthercatBusBase.hpp>
#include <soem_interface_rsl/EthercatSlaveBase.hpp>

#include <soem_rsl/ethercat.h>

namespace soem_interface_rsl {

static bool busIsAvailable(const std::string& name) {
  ec_adaptert* adapter = ec_find_adapters();
  while (adapter != nullptr) {
    if (name == std::string(adapter->name)) {
      return true;
    }
    adapter = adapter->next;
  }
  return false;
}

struct EthercatBusBaseTemplateAdapter::EthercatSlaveBaseImpl {
  EthercatSlaveBaseImpl() = delete;
  explicit EthercatSlaveBaseImpl(const std::string name) : name_(name), wkc_(0) {
    // Initialize all soem_rsl context data pointers that are not used with null.
    ecatContext_.elist->head = 0;
    ecatContext_.elist->tail = 0;
    ecatContext_.port->stack.sock = nullptr;
    ecatContext_.port->stack.txbuf = nullptr;
    ecatContext_.port->stack.txbuflength = nullptr;
    ecatContext_.port->stack.tempbuf = nullptr;
    ecatContext_.port->stack.rxbuf = nullptr;
    ecatContext_.port->stack.rxbufstat = nullptr;
    ecatContext_.port->stack.rxsa = nullptr;
    ecatContext_.port->redport = nullptr;
    //  ecatContext_.idxstack->data = nullptr; // This does not compile since soem_rsl uses a fixed size array of void pointers.
    ecatContext_.FOEhook = nullptr;
  }

  const std::string& getName() const { return name_; }

  bool busIsAvailable() const { return soem_interface_rsl::busIsAvailable(name_); }

  int getNumberOfSlaves() const {
    if (!initlialized_) {
      MELO_WARN_STREAM("[SOEM_Interface] requesting number of slaves on not inited bus.")
      return 0;
    }
    std::lock_guard<std::mutex> contextLock(contextMutex_);
    return *ecatContext_.slavecount;
  }

  bool addSlave(const EthercatSlaveBasePtr& slave) {
    for (const auto& existingSlave : slaves_) {
      if (slave->getAddress() == existingSlave->getAddress()) {
        MELO_ERROR_STREAM("[" << name_ << "] "
                              << "Slave '" << existingSlave->getName() << "' and slave '" << slave->getName()
                              << "' have identical addresses (" << slave->getAddress() << ").");
        return false;
      }
    }

    slaves_.push_back(slave);
    // ensure that they are sorted in adress order. this makes access simpler (access via slaveaddress -1)
    std::sort(slaves_.begin(), slaves_.end(),
              [](const EthercatSlaveBasePtr& a, const EthercatSlaveBasePtr& b) -> bool { return a->getAddress() < b->getAddress(); });
    return true;
  }

  bool startup(std::atomic<bool>& abortFlag, const bool sizeCheck, int maxDiscoverRetries) {
    /*
     * Followed by start of the application we need to set up the NIC to be used as
     * EtherCAT Ethernet interface. In a simple setup we call ec_init(ifname) and if
     * soem_rsl comes with support for cable redundancy we call ec_init_redundant that
     * will open a second port as backup. You can send NULL as ifname if you have a
     * dedicated NIC selected in the nicdrv.c. It returns >0 if succeeded.
     */

    if (!busIsAvailable()) {
      MELO_ERROR_STREAM("[" << name_ << "] "
                            << "Bus is not available.");
      EthercatBusBase::printAvailableBusses();
      return false;
    }

    {
      std::lock_guard<std::mutex> contextLock(contextMutex_);
      if (ecx_init(&ecatContext_, name_.c_str()) <= 0) {
        MELO_ERROR_STREAM("[" << name_ << "] "
                              << "No socket connection. Execute as root.");
        return false;
      }
      for (int retry = 0; retry <= maxDiscoverRetries; retry++) {
        if (abortFlag) {
          MELO_WARN_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                   << "Shutdown during waiting for slaves.");
          ecx_close(&ecatContext_);
          return false;  // avoid that executation continues.
        }
        if (ecx_detect_slaves(&ecatContext_) >= static_cast<int>(slaves_.size())) {
          // on some of the older (rsl) anydrives there seems to be a short race between bus is responsive and slave is fully ready...
          // so give them this 1 sec to be fully ready to be started...
          soem_interface_rsl::threadSleep(1.0);
          break;
        }
        if (retry == maxDiscoverRetries) {
          MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                    << "No slaves have been found.");
          ecx_close(&ecatContext_);
          return false;
        }
        // Sleep and retry.
        soem_interface_rsl::threadSleep(ecatConfigRetrySleep_);
        MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] No slaves have been found, retrying " << retry + 1 << "/"
                                                 << maxDiscoverRetries << " ...");
      }

      // this should no work cleanly, since we're sure that all slaves are started.
      if (ecx_config_init(&ecatContext_, FALSE) < static_cast<int>(slaves_.size())) {
        ecx_close(&ecatContext_);
        MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                  << "No slaves have been found.");
        return false;
      }

      int nSlaves = *ecatContext_.slavecount;
      // Print the slaves which have been detected.
      MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] The following " << nSlaves << " slaves have been found and configured:");
      for (int slave = 1; slave <= nSlaves; slave++) {
        MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Address: " << slave << " - Name: '"
                                                 << std::string(ecatContext_.slavelist[slave].name) << "'");
      }

      // Check if the given slave addresses are valid.
      bool slaveAddressesAreOk = true;
      for (const auto& slave : slaves_) {
        auto address = static_cast<int>(slave->getAddress());
        if (address == 0) {
          MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                    << "Slave '" << slave->getName() << "': Invalid address " << address << ".");
          slaveAddressesAreOk = false;
        }
        if (address > nSlaves) {
          MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                    << "Slave '" << slave->getName() << "': Invalid address " << address << ", "
                                                    << "only " << nSlaves << " slave(s) found.");
          slaveAddressesAreOk = false;
        }
      }
      if (!slaveAddressesAreOk) {
        ecx_close(&ecatContext_);
        return false;
      }

      // Disable symmetrical transfers.
      ecatContext_.grouplist[0].blockLRW = 1;

      // some slave might require SAFE_OP during setup...
      busDiagnosisLog_.errorCounters_.resize(slaves_.size());
      nSlaves_ = slaves_.size();
      initlialized_ = true;
      setStateLocked(EC_STATE_PRE_OP);
      waitForStateLocked(EC_STATE_PRE_OP, 0);
    }
    //  MELO_DEBUG_STREAM("[EthercatBus] Bus Startup: Set all salves to SAFE_OP")

    // Initialize the communication interfaces of all slaves.
    for (auto& slave : slaves_) {
      MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Starting slave: " << slave->getName())
      if (!slave->startup()) {
        MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] Slave '" << slave->getName() << "' was not initialized successfully.");
        return false;
      } else {
        MELO_DEBUG_STREAM("[soem_interface_rsl::" << name_ << "] Successfully started slave: " << slave->getName())
      }
    }

    std::lock_guard<std::mutex> contextLock(contextMutex_);
    // Set up the communication IO mapping.
    // Note: ecx_config_map_group(..) requests the slaves to go to SAFE-OP.
    [[maybe_unused]] int ioMapSize = ecx_config_map_group(&ecatContext_, &ioMap_, 0);
    MELO_DEBUG_STREAM("[soem_interface_rsl::" << name_ << "] Configured ioMap with size: " << ioMapSize)

    // Check if the size of the IO mapping fits our slaves.
    bool ioMapIsOk = true;
    // do this check only if 'sizeCheck' is true
    if (sizeCheck) {
      for (const auto& slave : slaves_) {
        const EthercatSlaveBase::PdoInfo pdoInfo = slave->getCurrentPdoInfo();
        if (pdoInfo.rxPdoSize_ != ecatContext_.slavelist[slave->getAddress()].Obytes) {
          MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                    << "RxPDO size mismatch: The slave '" << slave->getName() << "' expects a size of "
                                                    << pdoInfo.rxPdoSize_ << " bytes but the slave found at its address "
                                                    << slave->getAddress() << " requests "
                                                    << ecatContext_.slavelist[slave->getAddress()].Obytes << " bytes).");
          ioMapIsOk = false;
        }
        if (pdoInfo.txPdoSize_ != ecatContext_.slavelist[slave->getAddress()].Ibytes) {
          MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                    << "TxPDO size mismatch: The slave '" << slave->getName() << "' expects a size of "
                                                    << pdoInfo.txPdoSize_ << " bytes but the slave found at its address "
                                                    << slave->getAddress() << " requests "
                                                    << ecatContext_.slavelist[slave->getAddress()].Ibytes << " bytes).");
          ioMapIsOk = false;
        }
      }
    }
    if (!ioMapIsOk) {
      return false;
    }

    // Initialize the memory with zeroes.
    for (int slave = 1; slave <= *ecatContext_.slavecount; slave++) {
      memset(ecatContext_.slavelist[slave].inputs, 0, ecatContext_.slavelist[slave].Ibytes);
      memset(ecatContext_.slavelist[slave].outputs, 0, ecatContext_.slavelist[slave].Obytes);
    }

    workingCounterTooLowCounter_ = 0;

    return true;
  }

  void updateRead() {
    if (!sentProcessData_) {
      MELO_DEBUG_STREAM("No process data to read.");
      return;
    }

    //! Receive the EtherCAT data.
    updateReadStamp_ = std::chrono::high_resolution_clock::now();
    {
      std::lock_guard<std::mutex> guard(contextMutex_);
      wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
    }
    sentProcessData_ = false;

    int expectedWorkingCounter = ecatContext_.grouplist[0].outputsWKC * 2 + ecatContext_.grouplist[0].inputsWKC;
    //! Check the working counter.
    if (wkc_ < expectedWorkingCounter) {
      ++workingCounterTooLowCounter_;
      MELO_DEBUG_STREAM("[soem_interface_rsl::" << name_ << "] Working counter too low counter: " << workingCounterTooLowCounter_)
      MELO_DEBUG_THROTTLE_STREAM(1.0, "[soem_interface_rsl::" << getName() << "] Update Read:" << this);
      MELO_WARN_STREAM("[soem_interface_rsl::" << name_ << "] Working counter is too low: " << wkc_.load() << " < "
                                               << expectedWorkingCounter << ", wkc's to low in a row: " << workingCounterTooLowCounter_);
      {
        std::lock_guard<std::mutex> guard(contextMutex_);
        MELO_WARN_STREAM("[soem_interface_rsl" << name_ << "] For all slaves alStatusCode: 0x" << std::setfill('0') << std::setw(8)
                                               << std::hex << ecatContext_.slavelist[0].ALstatuscode << " "
                                               << ec_ALstatuscode2string(ecatContext_.slavelist[0].ALstatuscode));
      }
      if (workingCounterTooLowCounter_ > maxWorkingCounterTooLow_) {
        MELO_ERROR_THROTTLE_STREAM(1.0, "[soem_interface_rsl" << name_ << "] Bus is not ok. Too many working counter too low in a row: "
                                                              << workingCounterTooLowCounter_)
      }
      return;
    }
    // Reset working counter too low counter.
    workingCounterTooLowCounter_ = 0;

    //! Each slave attached to this bus reads its data to the buffer.
    for (auto& slave : slaves_) {
      slave->updateRead();
    }
  }

  void updateWrite() {
    if (sentProcessData_) {
      MELO_DEBUG_STREAM("[soem_interface_rsl] Sending new process data without reading the previous one.");
    }

    //! Each slave attached to this bus write its data to the buffer.
    for (auto& slave : slaves_) {
      slave->updateWrite();
    }

    //! Send the EtherCAT data.
    updateWriteStamp_ = std::chrono::high_resolution_clock::now();
    std::lock_guard<std::mutex> guard(contextMutex_);
    ecx_send_processdata(&ecatContext_);
    sentProcessData_ = true;
  }

  const std::chrono::time_point<std::chrono::high_resolution_clock>& getUpdateReadStamp() const { return updateReadStamp_; }

  const std::chrono::time_point<std::chrono::high_resolution_clock>& getUpateWriteStamp() const { return updateWriteStamp_; }

  void shutdown() {
    if (initlialized_) {
      {
        std::lock_guard<std::mutex> guard(contextMutex_);
        // Set the slaves to state Init.
        if (*ecatContext_.slavecount > 0) {
          setStateLocked(EC_STATE_INIT);
          waitForStateLocked(EC_STATE_INIT);
        }
      }  // release the contextMutex_ in case slave wants to do low_level commands at shutdown.
      for (auto& slave : slaves_) {
        slave->shutdown();
      }
    }

    // Close the port.
    std::lock_guard<std::mutex> guard(contextMutex_);
    if (ecatContext_.port != nullptr) {
      MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Closing socket ...");
      ecx_close(&ecatContext_);
      // Sleep to make sure the socket is closed, because ecx_close is non-blocking.
      soem_interface_rsl::threadSleep(0.5);
    }
    initlialized_ = false;
  }

  void setState(const uint16_t state, const uint16_t slave = 0) {
    std::lock_guard<std::mutex> guard(contextMutex_);
    setStateLocked(state, slave);
  }

  bool waitForState(const uint16_t state, const uint16_t slave = 0, const unsigned int maxRetries = 20) {
    std::lock_guard<std::mutex> guard(contextMutex_);
    return waitForStateLocked(state, slave, maxRetries);
  }

  ETHERCAT_SM_STATE getEthercatState(const uint16_t slave = 0) {
    uint16_t stateRaw = getState(slave);
    // check if Error
    if ((stateRaw & 0xf0) == EC_STATE_ERROR) {
      return ETHERCAT_SM_STATE::ERROR;
    }
    switch (stateRaw & 0x0f) {
      case EC_STATE_INIT:
        return ETHERCAT_SM_STATE::INIT;
      case EC_STATE_BOOT:
        return ETHERCAT_SM_STATE::BOOT;
      case EC_STATE_PRE_OP:
        return ETHERCAT_SM_STATE::PRE_OP;
      case EC_STATE_SAFE_OP:
        return ETHERCAT_SM_STATE::SAFE_OP;
      case EC_STATE_OPERATIONAL:
        return ETHERCAT_SM_STATE::OPERATIONAL;
      default:
        return ETHERCAT_SM_STATE::NONE;  // should not happen.
    }
    return ETHERCAT_SM_STATE::NONE;
  }

  bool busIsOk() const { return workingCounterTooLowCounter_ < maxWorkingCounterTooLow_; }

  bool doBusMonitoring(bool logErrorCounterForDiagnosis) {
    if (!initlialized_) {
      return false;
    }
    bool allFine = true;
    busDiagnosisLog_.fullyUpdated = false;
    BusDiagState nextBusDiagState{BusDiagState::StateReading};
    MELO_DEBUG_STREAM("[DriveManager::DoBusMonitoring::" << name_ << "] Running Bus Monitoring/Diagnosis")

    if (busDiagState_ == BusDiagState::StateReading) {
      if (logErrorCounterForDiagnosis) {
        nextBusDiagState = BusDiagState::CounterReading;
      }
      // read all the states from all slaves.
      MELO_DEBUG_STREAM("[DriveManager::DoBusMonitoring::" << name_ << "] Running Bus Monitoring/Diagnosis State/AlstatusCode")

      int lowestSlaveState = getState(0);  // one datagram iff all slaves in the same state, otherwise one datagram per slave.

      // can we do more than looking on the state machine? error counters would be interessting but needs very raw register reads, but
      // possible.
      std::lock_guard<std::mutex> guard(contextMutex_);
      if ((lowestSlaveState & 0x0f) < EC_STATE_OPERATIONAL) {  // if ECAT Error bus state is e.g. 0x14 = 0x10 (error) + 0x04 (safeOP)
        MELO_WARN_STREAM("[EthercatBus::BusMonitoring::" << name_ << "] No all slaves in EC_STATE_OPERATIONAL")
        for (const auto& slave : slaves_) {
          MELO_WARN_STREAM("[EthercatBus::BusMonitoring::"
                           << name_ << "] Slave: " << slave->getName()
                           << " in state: " << EthercatBusBase::getStateString(ecatContext_.slavelist[slave->getAddress()].state))

          if ((ecatContext_.slavelist[slave->getAddress()].state & 0x0f) < EC_STATE_OPERATIONAL) {
            MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slave->getName() << " alStatusCode: 0x" << std::setfill('0')
                                                     << std::setw(8) << std::hex << ecatContext_.slavelist[slave->getAddress()].ALstatuscode
                                                     << " "
                                                     << ec_ALstatuscode2string(ecatContext_.slavelist[slave->getAddress()].ALstatuscode));

            if (ecatContext_.slavelist[slave->getAddress()].state == EC_STATE_NONE && !ecatContext_.slavelist[slave->getAddress()].islost) {
              ecatContext_.slavelist[slave->getAddress()].islost = TRUE;
              MELO_ERROR_STREAM("[EthercatBus::BusMonitoring] Slave: "
                                << slave->getName() << " no valid state read - slave probably lost - check your cables ;-) !")
              // todo  Trying to recover the lost slave. !NOT IMPLEMENTED! example: in soem_rsl simple_test.c
              // slave (sdks) would require an optional virtual method, something like: slave->recover() in case they loose connection.
              // (could fix partially shacky cables in software..)
            }
          }
        }
        allFine = false;
      }
    }

    // only reached if errorCounterDiagnosis enabled.
    if (busDiagState_ == BusDiagState::CounterReading) {
      MELO_DEBUG_STREAM(
          "[DriveManager::DoBusMonitoring::" << name_ << "] Running Bus Monitoring/Diagnosis counter slave no: " << busDiagOfCurrentSlave_)
      nextBusDiagState = BusDiagState::StateReading;
      auto& selectedSlave = slaves_[busDiagOfCurrentSlave_];
      // Reading from registers: e.g. ECT_REG_RXERR, addr_0x0300 - 0x0307 (check Ethercat Specification ETG1000.4 for details)
      // one frame per slave.( BRD call would OR together the error counters, therefore for exact number one frame per slave required.)
      // we therefore continously update our saved values by mixing in a single datagram per call to this function and sweeping over the
      // slaves.
      std::byte rawData[REG::ERROR_COUNTERS_LIST.memorySize()];
      memset(rawData, 0xbe, REG::ERROR_COUNTERS_LIST.memorySize());
      std::lock_guard<std::mutex> guard(contextMutex_);
      if (ecx_FPRD(ecatContext_.port, ecatContext_.slavelist[selectedSlave->getAddress()].configadr,
                   static_cast<uint16_t>(REG::ERROR_COUNTERS::FRAME_ERROR_PORT0_ADDR), REG::ERROR_COUNTERS_LIST.memorySize(), rawData,
                   EC_TIMEOUTRET3)) {
        size_t currentRegNo{0};
        for (const auto& reg : REG::ERROR_COUNTERS_LIST) {
          uint8_t value = REG::ERROR_COUNTERS_LIST.getValueFromRawAs<uint8_t>(reg.addrEnum, rawData, REG::ERROR_COUNTERS_LIST.memorySize());
          if (busDiagnosisLog_.errorCounters_[busDiagOfCurrentSlave_][currentRegNo].previousValue > value) {
            // we had an overflow, (or multiple..) lets assume it was one, we just have to diagnose fast enough..
            busDiagnosisLog_.errorCounters_[busDiagOfCurrentSlave_][currentRegNo].fullValue +=
                (255 - busDiagnosisLog_.errorCounters_[busDiagOfCurrentSlave_][currentRegNo].previousValue) + value;
          } else {
            busDiagnosisLog_.errorCounters_[busDiagOfCurrentSlave_][currentRegNo].fullValue += value;
          }
          currentRegNo++;
        }
      } else {
        MELO_WARN_STREAM(
            "[soem_interface_rsl::BusMonitoring::" << name_ << "] Could not read Error counters for slave: " << selectedSlave->getName())
      }
      busDiagOfCurrentSlave_++;
      if (busDiagOfCurrentSlave_ >= nSlaves_) {
        busDiagOfCurrentSlave_ = 0;
        busDiagnosisLog_.fullyUpdated = true;
      }
    }
    busDiagState_ = nextBusDiagState;
    return allFine;
  }

  bool getBusDiagnosisLog(BusDiagnosisLog& busDiagnosisLogOut) {
    if (busDiagnosisLog_.fullyUpdated) {
      // is called in the update loop, therefore not thread safe implemented here..
      busDiagnosisLogOut = busDiagnosisLog_;
      return true;
    }
    return false;
  }

  void syncDistributedClock0(const uint16_t slave, const bool activate, const double cycleTime, const double cycleShift) {
    // todo verify!
    MELO_INFO_STREAM("Bus '" << name_ << "', slave " << slave << ":  " << (activate ? "Activating" : "Deactivating")
                             << " distributed clock synchronization...");

    ecx_dcsync0(&ecatContext_, slave, static_cast<uint8_t>(activate), static_cast<uint32_t>(cycleTime * 1e9),
                static_cast<int32_t>(1e9 * cycleShift));

    MELO_INFO_STREAM("Bus '" << name_ << "', slave " << slave << ":  " << (activate ? "Activated" : "Deactivated")
                             << " distributed clock synchronization.");
  }

  EthercatBusBase::PdoSizePair getHardwarePdoSizes(const uint16_t slave) {
    std::lock_guard<std::mutex> guard(contextMutex_);
    return std::make_pair(ecatContext_.slavelist[slave].Obytes, ecatContext_.slavelist[slave].Ibytes);
  }

  EthercatBusBase::PdoSizeMap getHardwarePdoSizes() {
    EthercatBusBase::PdoSizeMap pdoMap;

    for (const auto& slave : slaves_) {
      pdoMap.insert(std::make_pair(slave->getName(), getHardwarePdoSizes(slave->getAddress())));
    }
    return pdoMap;
  }

  bool sdoWrite(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, int size, void* buf) {
    int wkc = 0;
    {
      assert(static_cast<int>(slave) <= *ecatContext_.slavecount);
      std::lock_guard<std::mutex> guard(contextMutex_);
      wkc = ecx_SDOwrite(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), size, buf, EC_TIMEOUTRXM);
    }
    if (wkc <= 0) {
      MELO_ERROR_STREAM("Slave " << slave << ": Working counter too low (" << wkc << ") for writing SDO (ID: 0x" << std::setfill('0')
                                 << std::setw(4) << std::hex << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
                                 << static_cast<uint16_t>(subindex) << ").");
      checkForSdoErrors(slave, index);
      if (slave == 0) {
        MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Worst AL status code of all slaves, alStatusCode: 0x" << std::setfill('0')
                                                 << std::setw(8) << std::hex << ecatContext_.slavelist[slave].ALstatuscode << " "
                                                 << ec_ALstatuscode2string(ecatContext_.slavelist[slave].ALstatuscode));
      } else {
        MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slaves_[slave - 1]->getName() << " alStatusCode: 0x"
                                                 << std::setfill('0') << std::setw(8) << std::hex
                                                 << ecatContext_.slavelist[slave].ALstatuscode << " "
                                                 << ec_ALstatuscode2string(ecatContext_.slavelist[slave].ALstatuscode));
      }
      return false;
    }
    return true;
  }

  bool sdoRead(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, int size, void* buf) {
    int requestedSize = size;
    int wkc = 0;
    {
      assert(static_cast<int>(slave) <= *ecatContext_.slavecount);
      std::lock_guard<std::mutex> guard(contextMutex_);
      wkc = ecx_SDOread(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), &size, buf, EC_TIMEOUTRXM);
    }
    if (wkc <= 0) {
      MELO_ERROR_STREAM("Slave " << slave << ": Working counter too low (" << wkc << ") for reading SDO (ID: 0x" << std::setfill('0')
                                 << std::setw(4) << std::hex << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
                                 << static_cast<uint16_t>(subindex) << ").");

      checkForSdoErrors(slave, index);
      if (slave == 0) {
        MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Worst AL status code of all slaves, alStatusCode: 0x" << std::setfill('0')
                                                 << std::setw(8) << std::hex << ecatContext_.slavelist[slave].ALstatuscode << " "
                                                 << ec_ALstatuscode2string(ecatContext_.slavelist[slave].ALstatuscode));
      } else {
        MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slaves_[slave - 1]->getName() << " alStatusCode: 0x"
                                                 << std::setfill('0') << std::setw(8) << std::hex
                                                 << ecatContext_.slavelist[slave].ALstatuscode << " "
                                                 << ec_ALstatuscode2string(ecatContext_.slavelist[slave].ALstatuscode));
      }
      return false;
    }
    if (size != requestedSize) {
      MELO_ERROR_STREAM("Slave " << slave << ": Size mismatch (expected " << requestedSize << " bytes, read " << size
                                 << " bytes) for reading SDO (ID: 0x" << std::setfill('0') << std::setw(4) << std::hex << index
                                 << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(subindex) << ").");
      return false;
    }
    return true;
  }

  int sdoReadSize(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, int size, void* buf) {
    int wkc = 0;
    {
      assert(static_cast<int>(slave) <= *ecatContext_.slavecount);
      std::lock_guard<std::mutex> guard(contextMutex_);
      wkc = ecx_SDOread(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), &size, buf, EC_TIMEOUTRXM);
    }
    if (wkc <= 0) {
      MELO_ERROR_STREAM("Slave " << slave << ": Working counter too low (" << wkc << ") for reading SDO (ID: 0x" << std::setfill('0')
                                 << std::setw(4) << std::hex << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
                                 << static_cast<uint16_t>(subindex) << ").");

      checkForSdoErrors(slave, index);
      if (slave == 0) {
        MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Worst AL status code of all slaves, alStatusCode: 0x" << std::setfill('0')
                                                 << std::setw(8) << std::hex << ecatContext_.slavelist[slave].ALstatuscode << " "
                                                 << ec_ALstatuscode2string(ecatContext_.slavelist[slave].ALstatuscode));
      } else {
        MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slaves_[slave - 1]->getName() << " alStatusCode: 0x"
                                                 << std::setfill('0') << std::setw(8) << std::hex
                                                 << ecatContext_.slavelist[slave].ALstatuscode << " "
                                                 << ec_ALstatuscode2string(ecatContext_.slavelist[slave].ALstatuscode));
      }
      return 0;
    }
    return size;
  }

  void readTxPdo(const uint16_t slave, int size, void* buf) const {
    assert(static_cast<int>(slave) <= *ecatContext_.slavecount);
    std::lock_guard<std::mutex> guard(contextMutex_);
    assert(size == (int)ecatContext_.slavelist[slave].Ibytes);
    memcpy(buf, ecatContext_.slavelist[slave].inputs, size);
  }

  void writeRxPdo(const uint16_t slave, int size, const void* buf) {
    assert(static_cast<int>(slave) <= *ecatContext_.slavecount);
    std::lock_guard<std::mutex> guard(contextMutex_);
    assert((int)ecatContext_.slavelist[slave].Obytes == size);
    memcpy(ecatContext_.slavelist[slave].outputs, buf, size);
  }

 private:
  uint16_t getState(const uint16_t slave) {
    std::lock_guard<std::mutex> guard(contextMutex_);
    int lowest_state = ecx_readstate(&ecatContext_);

    // ecx_readstate updates reads all slaves in worst case with one datagram per slaves, and or's all the ALStatusCodes.
    // therefore we update here the bus AL StatusCode which is the OR of all slave's ALStatusCode!
    busDiagnosisLog_.ecatApplicationLayerStatus = ecatContext_.slavelist[0].ALstatuscode;
    if (slave == 0) {
      return lowest_state;
    }
    return ecatContext_.slavelist[slave].state;
  }

  void setStateLocked(const uint16_t state, const uint16_t slave = 0) {
    if (!initlialized_) {
      MELO_WARN_STREAM("[soem_interface_rsl::" << name_ << "] Bus " << name_ << " was not successfully initialized, skipping operation");
      return;
    }
    ecatContext_.slavelist[slave].state = state;
    if (state == EC_STATE_OPERATIONAL) {
      ecx_send_processdata(&ecatContext_);
      wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
    }
    ecx_writestate(&ecatContext_, slave);
    if (slave == 0) {
      MELO_DEBUG_STREAM("[soem_interface_rsl::" << name_ << "] All slaves on State " << EthercatBusBase::getStateString(state)
                                                << " has been set.");
    } else {
      MELO_DEBUG_STREAM("[soem_interface_rsl::" << name_ << "] Slave " << slaves_[slave - 1]->getName() << " State "
                                                << EthercatBusBase::getStateString(state) << " has been set.");
    }
  }

  bool waitForStateLocked(const uint16_t state, const uint16_t slave = 0, const unsigned int maxRetries = 20) {
    if (!initlialized_) {
      MELO_WARN_STREAM("[soem_interface_rsl::" << name_ << "] Bus " << name_ << " was not successfully initialized, skipping operation");
      return false;
    }
    uint16_t returnedState = 0;
    uint16_t currentState = ecx_statecheck(&ecatContext_, slave, state, 20000);
    if (currentState == state) {
      MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slave << ": State " << EthercatBusBase::getStateString(state)
                                               << " has been reached directly")
      return true;
    }
    for (unsigned int retry = 0; retry <= maxRetries; retry++) {
      int timeout = EC_TIMEOUTSTATE;
      switch (static_cast<ec_state>(state)) {
        case EC_STATE_NONE:
        case EC_STATE_INIT:
        case EC_STATE_PRE_OP:
        case EC_STATE_BOOT:
          break;
        case EC_STATE_SAFE_OP:
          timeout = EC_TIMEOUTSTATE * 4;
          break;
        case EC_STATE_OPERATIONAL:
          ecx_send_processdata(&ecatContext_);
          wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
          timeout = 20000;
          break;
        case EC_STATE_ACK:
          break;
      }
      returnedState = ecx_statecheck(&ecatContext_, slave, state, timeout);
      if (returnedState == state) {
        MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slave << ": State " << EthercatBusBase::getStateString(state)
                                                 << " has been reached after " << retry << " retries");
        return true;
      }
    }
    MELO_WARN_STREAM("[soem_interface_rsl::" << name_ << "] Slave " << slave << ": Targetstate " << EthercatBusBase::getStateString(state)
                                             << " has not been reached. Current State: " << returnedState);

    if (slave == 0) {
      MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Worst AL status code of all slaves, alStatusCode: 0x" << std::setfill('0')
                                               << std::setw(8) << std::hex << ecatContext_.slavelist[slave].ALstatuscode << " "
                                               << ec_ALstatuscode2string(ecatContext_.slavelist[slave].ALstatuscode));
    } else {
      MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slaves_[slave - 1]->getName() << " alStatusCode: 0x"
                                               << std::setfill('0') << std::setw(8) << std::hex
                                               << ecatContext_.slavelist[slave].ALstatuscode << " "
                                               << ec_ALstatuscode2string(ecatContext_.slavelist[slave].ALstatuscode));
    }
    return false;
  }

  std::string getErrorString(ec_errort error) {
    std::stringstream stream;
    stream << "Time: " << (static_cast<double>(error.Time.sec) + (static_cast<double>(error.Time.usec) / 1000000.0));

    switch (error.Etype) {
      case EC_ERR_TYPE_SDO_ERROR:
        stream << " SDO slave: " << error.Slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.Index << "."
               << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(error.SubIdx) << " error: 0x" << std::setfill('0')
               << std::setw(8) << std::hex << static_cast<unsigned>(error.AbortCode) << " " << ec_sdoerror2string(error.AbortCode);
        break;
      case EC_ERR_TYPE_EMERGENCY:
        stream << " EMERGENCY slave: " << error.Slave << " error: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.ErrorCode;
        break;
      case EC_ERR_TYPE_PACKET_ERROR:
        stream << " PACKET slave: " << error.Slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.Index << "."
               << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(error.SubIdx) << " error: 0x" << std::setfill('0')
               << std::setw(8) << std::hex << error.ErrorCode;
        break;
      case EC_ERR_TYPE_SDOINFO_ERROR:
        stream << " SDO slave: " << error.Slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.Index << "."
               << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(error.SubIdx) << " error: 0x" << std::setfill('0')
               << std::setw(8) << std::hex << static_cast<unsigned>(error.AbortCode) << " " << ec_sdoerror2string(error.AbortCode);
        break;
      case EC_ERR_TYPE_SOE_ERROR:
        stream << " SoE slave: " << error.Slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.Index
               << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex << static_cast<unsigned>(error.AbortCode) << " "
               << ec_soeerror2string(error.ErrorCode);
        break;
      case EC_ERR_TYPE_MBX_ERROR:
        stream << " MBX slave: " << error.Slave << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex << error.ErrorCode << " "
               << ec_mbxerror2string(error.ErrorCode);
        break;
      default:
        stream << " MBX slave: " << error.Slave << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex
               << static_cast<unsigned>(error.AbortCode);
        break;
    }
    return stream.str();
  }

  /*!
   * Check if an error for the SDO index of the slave exists.
   * @param slave   Address of the slave.
   * @param index   Index of the SDO.
   * @return True if an error for the index exists.
   */
  bool checkForSdoErrors(const uint16_t slave, const uint16_t index) {
    while (ecx_iserror(&ecatContext_)) {
      ec_errort error;
      if (ecx_poperror(&ecatContext_, &error)) {
        std::string errorStr = getErrorString(error);
        MELO_ERROR_STREAM(errorStr);
        if (error.Slave == slave && error.Index == index) {
          soem_interface_rsl::common::MessageLog::insertMessage(message_logger::log::levels::Level::Error, errorStr);
          return true;
        }
      }
    }
    return false;
  }

  //! Name of the bus.
  std::string name_;

  //! Whether the bus has been initialized successfully
  bool initlialized_{false};

  //! List of slaves.
  std::vector<EthercatSlaveBasePtr> slaves_;

  //! Bool indicating whether PDO data has been sent and not read yet.
  bool sentProcessData_{false};

  //! Working counter of the most recent PDO.
  std::atomic<int> wkc_;

  //! Time of the last successful PDO reading.
  std::chrono::time_point<std::chrono::high_resolution_clock> updateReadStamp_;
  //! Time of the last successful PDO writing.
  std::chrono::time_point<std::chrono::high_resolution_clock> updateWriteStamp_;

  //! Time to sleep between the retries.
  const double ecatConfigRetrySleep_{1.0};

  //! Count working counter too low in a row.
  unsigned int workingCounterTooLowCounter_{0};
  //! Maximal number of working counter to low.
  const unsigned int maxWorkingCounterTooLow_{100};

  //! Bus Diagnosis Counters, and dl status log
  BusDiagnosisLog busDiagnosisLog_{};
  enum class BusDiagState { StateReading = 0, CounterReading = 1 };
  BusDiagState busDiagState_{BusDiagState::StateReading};
  size_t nSlaves_{0};                // number of slaves on the bus - set after startup.
  size_t busDiagOfCurrentSlave_{0};  // running variable to send only one frame per slave.

  // EtherCAT input/output mapping of the slaves within the datagrams.
  char ioMap_[4096];

  // EtherCAT context data elements:

  // Port reference.
  ecx_portt ecatPort_;
  // List of slave data. Index 0 is reserved for the master, higher indices for the slaves.
  ec_slavet ecatSlavelist_[EC_MAXSLAVE];
  // Number of slaves found in the network.
  int ecatSlavecount_{0};
  // Slave group structure.
  ec_groupt ecatGrouplist_[EC_MAXGROUP];
  // Internal, reference to EEPROM cache buffer.
  uint8 ecatEsiBuf_[EC_MAXEEPBUF];
  // Internal, reference to EEPROM cache map.
  uint32 ecatEsiMap_[EC_MAXEEPBITMAP];
  // Internal, reference to error list.
  ec_eringt ecatEList_;
  // Internal, reference to processdata stack buffer info.
  ec_idxstackT ecatIdxStack_;
  // Boolean indicating if an error is available in error stack.
  boolean ecatError_{FALSE};
  // Reference to last DC time from slaves.
  int64 ecatDcTime_{0};
  // Internal, SM buffer.
  ec_SMcommtypet ecatSmCommtype_[EC_MAX_MAPT];
  // Internal, PDO assign list.
  ec_PDOassignt ecatPdoAssign_[EC_MAX_MAPT];
  // Internal, PDO description list.
  ec_PDOdesct ecatPdoDesc_[EC_MAX_MAPT];
  // Internal, SM list from EEPROM.
  ec_eepromSMt ecatSm_;
  // Internal, FMMU list from EEPROM.
  ec_eepromFMMUt ecatFmmu_;

  mutable std::mutex contextMutex_;

  // EtherCAT context data.
  // Note: soem_rsl does not use dynamic memory allocation (new/delete). Therefore
  // all context pointers must be null or point to an existing member.
  ecx_contextt ecatContext_ = {&ecatPort_,
                               &ecatSlavelist_[0],
                               &ecatSlavecount_,
                               EC_MAXSLAVE,
                               &ecatGrouplist_[0],
                               EC_MAXGROUP,
                               &ecatEsiBuf_[0],
                               &ecatEsiMap_[0],
                               0,
                               &ecatEList_,
                               &ecatIdxStack_,
                               &ecatError_,
                               &ecatDcTime_,
                               &ecatSmCommtype_[0],
                               &ecatPdoAssign_[0],
                               &ecatPdoDesc_[0],
                               &ecatSm_,
                               &ecatFmmu_,
                               nullptr,
                               nullptr,
                               0};
};

EthercatBusBaseTemplateAdapter::EthercatBusBaseTemplateAdapter(const std::string& name)
    : pImpl_(std::make_unique<EthercatSlaveBaseImpl>(name)) {}

// has to be defined in cpp! otherwise EthercatBusBaseTemplateAdapter is incomplete type.
EthercatBusBaseTemplateAdapter::~EthercatBusBaseTemplateAdapter() = default;

bool EthercatBusBaseTemplateAdapter::sdoWriteForward(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                                     const bool completeAccess, int size, void* buf) {
  return pImpl_->sdoWrite(slave, index, subindex, completeAccess, size, buf);
}

bool EthercatBusBaseTemplateAdapter::sdoReadForward(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                                    const bool completeAccess, int size, void* buf) {
  return pImpl_->sdoRead(slave, index, subindex, completeAccess, size, buf);
}

int EthercatBusBaseTemplateAdapter::sdoReadSizeForward(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                                       const bool completeAccess, int size, void* buf) {
  return pImpl_->sdoReadSize(slave, index, subindex, completeAccess, size, buf);
}

void EthercatBusBaseTemplateAdapter::readTxPdoForward(const uint16_t slave, int size, void* buf) const {
  pImpl_->readTxPdo(slave, size, buf);
}

void EthercatBusBaseTemplateAdapter::writeRxPdoForward(const uint16_t slave, int size, const void* buf) {
  pImpl_->writeRxPdo(slave, size, buf);
}

//***************************

EthercatBusBase::EthercatBusBase(const std::string& name) : EthercatBusBaseTemplateAdapter(name) {}

EthercatBusBase::~EthercatBusBase() = default;

bool EthercatBusBase::busIsAvailable(const std::string& name) {
  return soem_interface_rsl::busIsAvailable(name);
}

void EthercatBusBase::printAvailableBusses() {
  MELO_INFO_STREAM("Available adapters:");
  ec_adaptert* adapter = ec_find_adapters();
  while (adapter != nullptr) {
    MELO_INFO_STREAM("- Name: '" << adapter->name << "', description: '" << adapter->desc << "'");
    adapter = adapter->next;
  }
}

const std::string& EthercatBusBase::getName() const {
  return pImpl_->getName();
}

bool EthercatBusBase::busIsAvailable() const {
  return pImpl_->busIsAvailable();
}

int EthercatBusBase::getNumberOfSlaves() const {
  return pImpl_->getNumberOfSlaves();
}

bool EthercatBusBase::addSlave(const EthercatSlaveBasePtr& slave) {
  return pImpl_->addSlave(slave);
}

bool EthercatBusBase::startup(const bool sizeCheck, int maxDiscoverRetries) {
  std::atomic<bool> tmpAtomicForStart{false};
  return pImpl_->startup(tmpAtomicForStart, sizeCheck, maxDiscoverRetries);
}

bool EthercatBusBase::startup(std::atomic<bool>& abortFlag, const bool sizeCheck, int maxDiscoverRetries) {
  return pImpl_->startup(abortFlag, sizeCheck, maxDiscoverRetries);
}

void EthercatBusBase::updateRead() {
  pImpl_->updateRead();
}

void EthercatBusBase::updateWrite() {
  pImpl_->updateWrite();
}

void EthercatBusBase::shutdown() {
  pImpl_->shutdown();
  pImpl_.reset(nullptr);
}

void EthercatBusBase::setState(const uint16_t state, const uint16_t slave) {
  pImpl_->setState(state, slave);
}

void EthercatBusBase::setState(soem_interface_rsl::ETHERCAT_SM_STATE state, const uint16_t slave) {
  pImpl_->setState(static_cast<uint16_t>(state), slave);
}

bool EthercatBusBase::waitForState(const uint16_t state, const uint16_t slave, const unsigned int maxRetries) {
  return pImpl_->waitForState(state, slave, maxRetries);
}
bool EthercatBusBase::waitForState(soem_interface_rsl::ETHERCAT_SM_STATE state, const uint16_t slave, const unsigned int maxRetries) {
  return pImpl_->waitForState(static_cast<uint16_t>(state), slave, maxRetries);
}

bool EthercatBusBase::busIsOk() const {
  return pImpl_->busIsOk();
}

std::string EthercatBusBase::getStateString(uint16_t state) {
  std::string stateStr{};
  switch (state & 0x0f) {
    case EC_STATE_INIT:
      stateStr += "EC_STATE_INIT";
      break;
    case EC_STATE_PRE_OP:
      stateStr += "EC_STATE_PRE_OP";
      break;
    case EC_STATE_SAFE_OP:
      stateStr += "EC_STATE_SAFE_OP";
      break;
    case EC_STATE_OPERATIONAL:
      stateStr += "EC_STATE_OPERATIONAL";
      break;
    case EC_STATE_BOOT:
      stateStr += "EC_STATE_BOOT";
      break;
    default:
      break;
  }
  if ((state & 0xf0) == EC_STATE_ERROR) {
    stateStr += " + EC_STATE_ERROR";
  }
  if ((state & 0xf0) == (EC_STATE_ERROR + EC_STATE_ACK)) {
    stateStr += " + EC_STATE_ERROR + EC_STATE_ACK";
  }
  return stateStr;
}

void EthercatBusBase::syncDistributedClock0(const uint16_t slave, const bool activate, const double cycleTime, const double cycleShift) {
  pImpl_->syncDistributedClock0(slave, activate, cycleTime, cycleShift);
}

EthercatBusBase::PdoSizeMap EthercatBusBase::getHardwarePdoSizes() {
  return pImpl_->getHardwarePdoSizes();
}

EthercatBusBase::PdoSizePair EthercatBusBase::getHardwarePdoSizes(const uint16_t slave) {
  return pImpl_->getHardwarePdoSizes(slave);
}

soem_interface_rsl::ETHERCAT_SM_STATE EthercatBusBase::getEthercatState(const uint16_t slave) {
  return pImpl_->getEthercatState(slave);
}

bool EthercatBusBase::doBusMonitoring(bool logErrorCounterForDiagnosis) {
  return pImpl_->doBusMonitoring(logErrorCounterForDiagnosis);
}

bool EthercatBusBase::getBusDiagnosisLog(BusDiagnosisLog& busDiagnosisLogOut) {
  return pImpl_->getBusDiagnosisLog(busDiagnosisLogOut);
}

bool EthercatBusBase::sendSdoReadVisibleString(const uint16_t slave, const uint16_t index, const uint8_t subindex, std::string& value) {
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  char buffer[128];
  int length = sizeof(buffer) - 1;

  int readLength = sdoReadSizeForward(slave, index, subindex, static_cast<boolean>(false), length, &buffer);
  if (readLength == 0) {
    return false;
  }

  value.clear();
  for (int i = 0; i < readLength; ++i) {
    if (buffer[i] != 0x0) {
      value += buffer[i];
    } else {
      break;
    }
  }
  return true;
}

const std::chrono::time_point<std::chrono::high_resolution_clock>& EthercatBusBase::getUpdateReadStamp() const {
  return pImpl_->getUpdateReadStamp();
}

const std::chrono::time_point<std::chrono::high_resolution_clock>& EthercatBusBase::getUpdateWriteStamp() const {
  return pImpl_->getUpateWriteStamp();
}

template <>
bool EthercatBusBase::sendSdoRead<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                               const bool completeAccess, std::string& value) {
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  // Expected length of the string. String needs to be preallocated
  int size = value.length();
  // Create buffer with the length of the string
  char buffer[size];
  bool success = sdoReadForward(slave, index, subindex, completeAccess, size, &buffer);
  value = std::string(buffer, size);
  return success;
}

template <>
bool EthercatBusBase::sendSdoWrite<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                                const bool completeAccess, const std::string value) {
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  const int size = value.length();
  std::string valueCopy{value};
  char* dataPtr = valueCopy.data();
  return sdoWriteForward(slave, index, subindex, completeAccess, size, dataPtr);
}

}  // namespace soem_interface_rsl
