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
#include <cassert>
#include <deque>
#include <iostream>
#include <mutex>

// message logger
#include <message_logger/message_logger.hpp>

namespace soem_interface {
namespace common {

class MessageLog {
 public:
  using Log = std::deque<std::pair<message_logger::log::levels::Level, std::string>>;

 protected:
  static constexpr size_t maxLogSize_ = 20;

  static std::mutex logMutex_;
  static Log log_;

 public:
  static void insertMessage(message_logger::log::levels::Level level, const std::string& message);
  static Log getLog();
  static void clearLog();
  static Log getAndClearLog();
};

}  // namespace common
}  // namespace soem_interface
