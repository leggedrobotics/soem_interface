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
#include "soem_interface/common/Macros.hpp"

namespace soem_interface {
namespace common {

std::mutex MessageLog::logMutex_;
MessageLog::Log MessageLog::log_;

void MessageLog::insertMessage(message_logger::log::levels::Level level, const std::string& message) {
  std::lock_guard<std::mutex> lock(logMutex_);
  log_.push_back({level, message});
  if (log_.size() > maxLogSize_) {
    log_.pop_front();
  }
}

MessageLog::Log MessageLog::getLog() {
  std::lock_guard<std::mutex> lock(logMutex_);
  return log_;
}

void MessageLog::clearLog() {
  std::lock_guard<std::mutex> lock(logMutex_);
  log_.clear();
}

MessageLog::Log MessageLog::getAndClearLog() {
  const Log log = getLog();
  clearLog();
  return log;
}

}  // namespace common
}  // namespace soem_interface
