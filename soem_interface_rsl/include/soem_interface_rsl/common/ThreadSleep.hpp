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
#define FOX_HELPER_DLL_IMPORT __attribute__((visibility("default")))
#define FOX_HELPER_DLL_EXPORT __attribute__((visibility("default")))
#define FOX_HELPER_DLL_LOCAL __attribute__((visibility("hidden")))
#define FOX_API FOX_HELPER_DLL_IMPORT
#define FOX_LOCAL FOX_HELPER_DLL_LOCAL

namespace soem_interface_rsl {

FOX_API void threadSleep(const double duration);

}  // namespace soem_interface_rsl
