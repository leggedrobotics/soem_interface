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

#include <soem_interface/EthercatBusBase.hpp>
#include <soem_interface/EthercatSlaveBase.hpp>

#define RX_PDO_ID 0x6000
#define TX_PDO_ID 0x7000

namespace soem_interface_examples {

struct TxPdo {
  uint8_t state = 0;
  float data1 = 0.0;
  float data2 = 0.0;
} __attribute__((packed));

struct RxPdo {
  float command1 = 0.0;
  float command2 = 0.0;
} __attribute__((packed));


class ExampleSlave : public soem_interface::EthercatSlaveBase {
public:
  ExampleSlave(const std::string& name, soem_interface::EthercatBusBase* bus, const uint32_t address);
  ~ExampleSlave() override = default;

  std::string getName() const override {
    return name_;
  }

  bool startup() override;
  void updateRead() override;
  void updateWrite() override;
  void shutdown() override;

  PdoInfo getCurrentPdoInfo() const override {
    return pdoInfo_;
  }

private:
  const std::string name_;
  PdoInfo pdoInfo_;
  TxPdo reading_;
  RxPdo command_;

};

}
