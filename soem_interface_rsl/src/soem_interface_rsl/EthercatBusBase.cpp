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

namespace soem_interface_rsl {

EthercatBusBase::EthercatBusBase(const std::string& name) : name_(name), wkc_(0) {
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

bool EthercatBusBase::busIsAvailable(const std::string& name) {
  ec_adaptert* adapter = ec_find_adapters();
  while (adapter != nullptr) {
    if (name == std::string(adapter->name)) {
      return true;
    }
    adapter = adapter->next;
  }
  return false;
}

void EthercatBusBase::printAvailableBusses() {
  MELO_INFO_STREAM("Available adapters:");
  ec_adaptert* adapter = ec_find_adapters();
  while (adapter != nullptr) {
    MELO_INFO_STREAM("- Name: '" << adapter->name << "', description: '" << adapter->desc << "'");
    adapter = adapter->next;
  }
}

bool EthercatBusBase::busIsAvailable() const {
  return busIsAvailable(name_);
}

int EthercatBusBase::getNumberOfSlaves() const {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  return *ecatContext_.slavecount;
}

bool EthercatBusBase::addSlave(const EthercatSlaveBasePtr& slave) {
  for (const auto& existingSlave : slaves_) {
    if (slave->getAddress() == existingSlave->getAddress()) {
      MELO_ERROR_STREAM("[" << getName() << "] "
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

bool EthercatBusBase::startup(const bool sizeCheck) {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  /*
   * Followed by start of the application we need to set up the NIC to be used as
   * EtherCAT Ethernet interface. In a simple setup we call ec_init(ifname) and if
   * soem_rsl comes with support for cable redundancy we call ec_init_redundant that
   * will open a second port as backup. You can send NULL as ifname if you have a
   * dedicated NIC selected in the nicdrv.c. It returns >0 if succeeded.
   */
  if (!busIsAvailable()) {
    MELO_ERROR_STREAM("[" << getName() << "] "
                          << "Bus is not available.");
    printAvailableBusses();
    return false;
  }
  if (ecx_init(&ecatContext_, name_.c_str()) <= 0) {
    MELO_ERROR_STREAM("[" << getName() << "] "
                          << "No socket connection. Execute as root.");
    return false;
  }

  // Initialize soem_rsl.
  // Note: ecx_config_init(..) requests the slaves to go to PRE-OP.
  for (unsigned int retry = 0; retry <= ecatConfigMaxRetries_; retry++) {
    if (ecx_config_init(&ecatContext_, FALSE) > 0) {
      // Successful initialization.
      break;
    } else if (retry == ecatConfigMaxRetries_) {
      // Too many failed attempts.
      MELO_ERROR_STREAM("[soem_interface_rsl::" << getName() << "] "
                                                << "No slaves have been found.");
      return false;
    }
    // Sleep and retry.
    soem_interface_rsl::threadSleep(ecatConfigRetrySleep_);
    MELO_INFO_STREAM("No slaves have been found, retrying " << retry + 1 << "/" << ecatConfigMaxRetries_ << " ...");
  }

  // Print the slaves which have been detected.
  MELO_INFO_STREAM("[soem_interface_rsl::" << getName() << "] The following " << getNumberOfSlaves()
                                           << " slaves have been found and configured:");
  for (int slave = 1; slave <= getNumberOfSlaves(); slave++) {
    MELO_INFO_STREAM("[soem_interface_rsl::" << getName() << "] Address: " << slave << " - Name: '"
                                             << std::string(ecatContext_.slavelist[slave].name) << "'");
  }

  // Check if the given slave addresses are valid.
  bool slaveAddressesAreOk = true;
  for (const auto& slave : slaves_) {
    auto address = static_cast<int>(slave->getAddress());
    if (address == 0) {
      MELO_ERROR_STREAM("[soem_interface_rsl::" << getName() << "] "
                                                << "Slave '" << slave->getName() << "': Invalid address " << address << ".");
      slaveAddressesAreOk = false;
    }
    if (address > getNumberOfSlaves()) {
      MELO_ERROR_STREAM("[soem_interface_rsl::" << getName() << "] "
                                                << "Slave '" << slave->getName() << "': Invalid address " << address << ", "
                                                << "only " << getNumberOfSlaves() << " slave(s) found.");
      slaveAddressesAreOk = false;
    }
  }
  if (!slaveAddressesAreOk) {
    return false;
  }

  // Disable symmetrical transfers.
  ecatContext_.grouplist[0].blockLRW = 1;

  // some slave might require SAFE_OP during setup...
  busDiagnosisLog_.errorCounters_.resize(slaves_.size());
  nSlaves_ = slaves_.size();
  initlialized_ = true;
  setState(EC_STATE_PRE_OP);
  waitForState(EC_STATE_PRE_OP, 0);
  //  MELO_DEBUG_STREAM("[EthercatBus] Bus Startup: Set all salves to SAFE_OP")

  // Initialize the communication interfaces of all slaves.
  for (auto& slave : slaves_) {
    MELO_DEBUG_STREAM("[soem_interface_rsl::" << name_ << "] Starting slave: " << slave->getName())
    if (!slave->startup()) {
      MELO_ERROR_STREAM("[soem_interface_rsl::" << getName() << "] Slave '" << slave->getName() << "' was not initialized successfully.");
      return false;
    } else {
      MELO_DEBUG_STREAM("[soem_interface_rsl::" << getName() << "] Successfully started slave: " << slave->getName())
    }
  }

  // Set up the communication IO mapping.
  // Note: ecx_config_map_group(..) requests the slaves to go to SAFE-OP.
  [[maybe_unused]] int ioMapSize = ecx_config_map_group(&ecatContext_, &ioMap_, 0);
  MELO_DEBUG_STREAM("[soem_interface_rsl::" << getName() << "] Configured ioMap with size: " << ioMapSize)

  // Check if the size of the IO mapping fits our slaves.
  bool ioMapIsOk = true;
  // do this check only if 'sizeCheck' is true
  if (sizeCheck) {
    for (const auto& slave : slaves_) {
      const EthercatSlaveBase::PdoInfo pdoInfo = slave->getCurrentPdoInfo();
      if (pdoInfo.rxPdoSize_ != ecatContext_.slavelist[slave->getAddress()].Obytes) {
        MELO_ERROR_STREAM("[soem_interface_rsl::" << getName() << "] "
                                                  << "RxPDO size mismatch: The slave '" << slave->getName() << "' expects a size of "
                                                  << pdoInfo.rxPdoSize_ << " bytes but the slave found at its address "
                                                  << slave->getAddress() << " requests "
                                                  << ecatContext_.slavelist[slave->getAddress()].Obytes << " bytes).");
        ioMapIsOk = false;
      }
      if (pdoInfo.txPdoSize_ != ecatContext_.slavelist[slave->getAddress()].Ibytes) {
        MELO_ERROR_STREAM("[soem_interface_rsl::" << getName() << "] "
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
  for (int slave = 1; slave <= getNumberOfSlaves(); slave++) {
    memset(ecatContext_.slavelist[slave].inputs, 0, ecatContext_.slavelist[slave].Ibytes);
    memset(ecatContext_.slavelist[slave].outputs, 0, ecatContext_.slavelist[slave].Obytes);
  }

  workingCounterTooLowCounter_ = 0;

  return true;
}

void EthercatBusBase::updateRead() {
  if (!sentProcessData_) {
    MELO_DEBUG_STREAM("No process data to read.");
    return;
  }

  //! Receive the EtherCAT data.
  updateReadStamp_ = std::chrono::high_resolution_clock::now();
  {
    std::lock_guard<std::recursive_mutex> guard(contextMutex_);
    wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
  }
  sentProcessData_ = false;

  //! Check the working counter.
  if (!workingCounterIsOk()) {
    ++workingCounterTooLowCounter_;
    MELO_DEBUG_STREAM("[soem_interface_rsl::" << name_ << "] Working counter too low counter: " << workingCounterTooLowCounter_)
    MELO_DEBUG_THROTTLE_STREAM(1.0, "[soem_interface_rsl::" << getName() << "] Update Read:" << this);
    MELO_WARN_STREAM("[soem_interface_rsl::" << name_ << "] Working counter is too low: " << wkc_.load() << " < "
                                             << getExpectedWorkingCounter() << ", wkc's to low in a row: " << workingCounterTooLowCounter_);
    {
      std::lock_guard<std::recursive_mutex> guard(contextMutex_);
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

void EthercatBusBase::updateWrite() {
  if (sentProcessData_) {
    MELO_DEBUG_STREAM("Sending new process data without reading the previous one.");
  }

  //! Each slave attached to this bus write its data to the buffer.
  for (auto& slave : slaves_) {
    slave->updateWrite();
  }

  //! Send the EtherCAT data.
  updateWriteStamp_ = std::chrono::high_resolution_clock::now();
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  ecx_send_processdata(&ecatContext_);
  sentProcessData_ = true;
}

void EthercatBusBase::shutdown() {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  // Set the slaves to state Init.
  if (getNumberOfSlaves() > 0) {
    setState(EC_STATE_INIT);
    waitForState(EC_STATE_INIT);
  }

  for (auto& slave : slaves_) {
    slave->shutdown();
  }

  // Close the port.
  if (ecatContext_.port != nullptr) {
    MELO_INFO_STREAM("Closing socket ...");
    ecx_close(&ecatContext_);
    // Sleep to make sure the socket is closed, because ecx_close is non-blocking.
    soem_interface_rsl::threadSleep(0.5);
  }

  initlialized_ = false;
}

void EthercatBusBase::setState(const uint16_t state, const uint16_t slave) {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  if (!initlialized_) {
    MELO_WARN_STREAM("[soem_interface_rsl::" << name_ << "] Bus " << name_ << " was not successfully initialized, skipping operation");
    return;
  }
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  ecatContext_.slavelist[slave].state = state;
  if (state == EC_STATE_OPERATIONAL) {
    ecx_send_processdata(&ecatContext_);
    wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
  }
  ecx_writestate(&ecatContext_, slave);
  if (slave == 0) {
    MELO_DEBUG_STREAM("[soem_interface_rsl::" << name_ << "] All slaves on State " << getStateString(state) << " has been set.");
  } else {
    MELO_DEBUG_STREAM("[soem_interface_rsl::" << name_ << "] Slave " << slaves_[slave - 1]->getName() << " State " << getStateString(state)
                                              << " has been set.");
  }
}

bool EthercatBusBase::waitForState(const uint16_t state, const uint16_t slave, const unsigned int maxRetries, const double retrySleep) {
  (void)retrySleep;  // unused.
  if (!initlialized_) {
    MELO_WARN_STREAM("[soem_interface_rsl::" << name_ << "] Bus " << name_ << " was not successfully initialized, skipping operation");
    return false;
  }
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  uint16_t returnedState = 0;
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  uint16_t currentState = ecx_statecheck(&ecatContext_, slave, state, 20000);
  if (currentState == state) {
    MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slave << ": State " << getStateString(state)
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
      MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slave << ": State " << getStateString(state)
                                               << " has been reached after " << retry << " retries");
      return true;
    }
  }
  MELO_WARN_STREAM("[soem_interface_rsl::" << name_ << "] Slave " << slave << ": Targetstate " << getStateString(state)
                                           << " has not been reached. Current State: " << returnedState);
  printALStatus(slave);
  return false;
}

int EthercatBusBase::getExpectedWorkingCounter(const uint16_t slave) const {
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  return ecatContext_.grouplist[slave].outputsWKC * 2 + ecatContext_.grouplist[slave].inputsWKC;
}

std::string EthercatBusBase::getErrorString(ec_errort error) {
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

void EthercatBusBase::printALStatus(const uint16_t slave) {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  if (slave == 0) {
    MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Worst AL status code of all slaves, alStatusCode: 0x" << std::setfill('0')
                                             << std::setw(8) << std::hex << ecatContext_.slavelist[slave].ALstatuscode << " "
                                             << ec_ALstatuscode2string(ecatContext_.slavelist[slave].ALstatuscode));
  } else {
    MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slaves_[slave - 1]->getName() << " alStatusCode: 0x"
                                             << std::setfill('0') << std::setw(8) << std::hex << ecatContext_.slavelist[slave].ALstatuscode
                                             << " " << ec_ALstatuscode2string(ecatContext_.slavelist[slave].ALstatuscode));
  }
}

void EthercatBusBase::printALStatus(const EthercatSlaveBasePtr& slave) {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  assert(static_cast<int>(slave->getAddress()) <= getNumberOfSlaves());
  MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slave->getName() << " alStatusCode: 0x" << std::setfill('0')
                                           << std::setw(8) << std::hex << ecatContext_.slavelist[slave->getAddress()].ALstatuscode << " "
                                           << ec_ALstatuscode2string(ecatContext_.slavelist[slave->getAddress()].ALstatuscode));
}

bool EthercatBusBase::checkForSdoErrors(const uint16_t slave, const uint16_t index) {
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

bool EthercatBusBase::workingCounterIsOk() const {
  return wkc_ >= getExpectedWorkingCounter();
}

bool EthercatBusBase::busIsOk() const {
  return workingCounterTooLowCounter_ < maxWorkingCounterTooLow_;
}

void EthercatBusBase::syncDistributedClock0(const uint16_t slave, const bool activate, const double cycleTime, const double cycleShift) {
  MELO_INFO_STREAM("Bus '" << name_ << "', slave " << slave << ":  " << (activate ? "Activating" : "Deactivating")
                           << " distributed clock synchronization...");

  ecx_dcsync0(&ecatContext_, slave, static_cast<uint8_t>(activate), static_cast<uint32_t>(cycleTime * 1e9),
              static_cast<int32_t>(1e9 * cycleShift));

  MELO_INFO_STREAM("Bus '" << name_ << "', slave " << slave << ":  " << (activate ? "Activated" : "Deactivated")
                           << " distributed clock synchronization.");
}

EthercatBusBase::PdoSizeMap EthercatBusBase::getHardwarePdoSizes() {
  PdoSizeMap pdoMap;

  for (const auto& slave : slaves_) {
    pdoMap.insert(std::make_pair(slave->getName(), getHardwarePdoSizes(slave->getAddress())));
  }

  return pdoMap;
}

EthercatBusBase::PdoSizePair EthercatBusBase::getHardwarePdoSizes(const uint16_t slave) {
  return std::make_pair(ecatContext_.slavelist[slave].Obytes, ecatContext_.slavelist[slave].Ibytes);
}

int EthercatBusBase::getState(const uint16_t slave) {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  int lowest_state = ecx_readstate(&ecatContext_);

  // ecx_readstate updates reads all slaves in worst case with one datagram per slaves, and or's all the ALStatusCodes.
  // therefore we update here the bus AL StatusCode which is the OR of all slave's ALStatusCode!
  busDiagnosisLog_.ecatApplicationLayerStatus = ecatContext_.slavelist[0].ALstatuscode;
  if (slave == 0) {
    return lowest_state;
  }
  return static_cast<int>(ecatContext_.slavelist[slave].state);
}

bool EthercatBusBase::doBusMonitoring(bool logErrorCounterForDiagnosis) {
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
    std::lock_guard<std::recursive_mutex> guard(contextMutex_);
    if ((lowestSlaveState & 0x0f) < EC_STATE_OPERATIONAL) {  // if ECAT Error bus state is e.g. 0x14 = 0x10 (error) + 0x04 (safeOP)
      MELO_WARN_STREAM("[EthercatBus::BusMonitoring::" << name_ << "] No all slaves in EC_STATE_OPERATIONAL")
      for (const auto& slave : slaves_) {
        MELO_WARN_STREAM("[EthercatBus::BusMonitoring::" << name_ << "] Slave: " << slave->getName() << " in state: "
                                                         << getStateString(ecatContext_.slavelist[slave->getAddress()].state))

        if ((ecatContext_.slavelist[slave->getAddress()].state & 0x0f) < EC_STATE_OPERATIONAL) {
          printALStatus(slave->getAddress());

          if (ecatContext_.slavelist[slave->getAddress()].state == EC_STATE_NONE && !ecatContext_.slavelist[slave->getAddress()].islost) {
            ecatContext_.slavelist[slave->getAddress()].islost = TRUE;
            MELO_ERROR_STREAM("[EthercatBus::BusMonitoring] Slave: "
                              << slave->getName() << " no valid state read - slave probably lost - check your cables ;-) !")
            // todo  Trying to recover the lost slave. !NOT IMPLEMENTED! example: in soem_rsl simple_test.c
            // slave (sdks) would require an optional virtual method, something like: slave->recover() in case they loose connection. (could
            // fix partially shacky cables in software..)
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
    std::lock_guard<std::recursive_mutex> guard(contextMutex_);
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
      MELO_WARN_STREAM("[soem_interface_rsl::BusMonitoring::" << name_
                                                              << "] Could not read Error counters for slave: " << selectedSlave->getName())
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

bool EthercatBusBase::getBusDiagnosisLog(BusDiagnosisLog& busDiagnosisLogOut) {
  if (busDiagnosisLog_.fullyUpdated) {
    // is called in the update loop, therefore not thread safe implemented here..
    busDiagnosisLogOut = busDiagnosisLog_;
    return true;
  }
  return false;
}

template <>
bool EthercatBusBase::sendSdoRead<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                               const bool completeAccess, std::string& value) {
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  // Expected length of the string. String needs to be preallocated
  int size = value.length();
  // Store for check at the end
  int expected_size = size;
  // Create buffer with the length of the string
  char buffer[size];
  int wkc = 0;
  {
    std::lock_guard<std::recursive_mutex> guard(contextMutex_);
    wkc = ecx_SDOread(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), &size, buffer, EC_TIMEOUTRXM);
    // Convert read data to a std::string
    value = std::string(buffer, size);
  }
  if (wkc <= 0) {
    MELO_ERROR_STREAM("Slave " << slave << ": Working counter too low (" << wkc << ") for reading SDO (ID: 0x" << std::setfill('0')
                               << std::setw(4) << std::hex << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
                               << static_cast<uint16_t>(subindex) << ").");
    return false;
  }

  if (size != (int)expected_size) {
    MELO_ERROR_STREAM("Slave " << slave << ": Size mismatch (expected " << expected_size << " bytes, read " << size
                               << " bytes) for reading SDO (ID: 0x" << std::setfill('0') << std::setw(4) << std::hex << index << ", SID 0x"
                               << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(subindex) << ").");
    return false;
  }
  return true;
}

template <>
bool EthercatBusBase::sendSdoWrite<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                                const bool completeAccess, const std::string value) {
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  const int size = value.length();
  const char* dataPtr = value.data();
  int wkc = 0;
  {
    std::lock_guard<std::recursive_mutex> guard(contextMutex_);
    wkc = ecx_SDOwrite(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), size, &dataPtr, EC_TIMEOUTRXM);
  }
  if (wkc <= 0) {
    MELO_ERROR_STREAM("Slave " << slave << ": Working counter too low (" << wkc << ") for writing SDO (ID: 0x" << std::setfill('0')
                               << std::setw(4) << std::hex << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
                               << static_cast<uint16_t>(subindex) << ").");
    return false;
  }
  return true;
}
}  // namespace soem_interface_rsl
