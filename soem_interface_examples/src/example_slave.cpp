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

#include <soem_interface_examples/ExampleSlave.hpp>
#include <soem_interface/EthercatBusBase.hpp>

// This shows a minimal example on how to use the soem_interface library. 
// Keep in mind that this is non-working example code, with only minimal error handling

int main(int argc, char** argv) {
  const std::string busName = "eth1";
  const std::string slaveName = "ExampleSlave";
  const uint32_t slaveAddress = 0;

  std::unique_ptr<soem_interface::EthercatBusBase> bus = std::make_unique<soem_interface::EthercatBusBase> (
    busName);

  std::shared_ptr<soem_interface_examples::ExampleSlave> slave = std::make_shared<soem_interface_examples::ExampleSlave> (
    slaveName, bus.get(), slaveAddress);

  bus->addSlave(slave);
  bus->startup();
  bus->setState(EC_STATE_OPERATIONAL);

  if(!bus->waitForState(EC_STATE_OPERATIONAL, slaveAddress)) {
    // Something is wrong
    return 1;
  }

  while(true) {
    bus->updateRead();
    bus->updateWrite();
  }

  bus->shutdown();
  return 0;
}
