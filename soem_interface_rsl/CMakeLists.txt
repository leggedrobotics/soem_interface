# Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
# Markus Staeuble, Jonas Junger, Johannes Pankert, Philipp Leemann,
# Tom Lankhorst, Samuel Bachmann, Gabriel Hottiger, Lennert Nachtigall,
# Mario Mauerer, Remo Diethelm
#
# This file is part of the soem_interface_rsl.
#
# The soem_interface_rsl is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# The seom_interface is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with the soem_interface_rsl.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.15)
project(soem_interface_rsl)

add_definitions(-std=c++17 -Wall -Werror)
add_definitions(-DMELO_USE_COUT)


set(PACKAGE_DEPENDENCIES
  ament_cmake
  message_logger
  soem_rsl
)

find_package(ament_cmake REQUIRED)
find_package(soem_rsl REQUIRED)
find_package(message_logger REQUIRED)


#this has to be a shared lib because we have to avoid symbol clashes with other (older) versions of SOEM which might exists in the workspace.
# e.g. ros-noetic-soem package (used by BOTA, ANYbotics.)
add_library(
  ${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/common/ThreadSleep.cpp
  src/${PROJECT_NAME}/common/Macros.cpp
  src/${PROJECT_NAME}/EthercatSlaveBase.cpp
  src/${PROJECT_NAME}/EthercatBusManagerBase.cpp
  src/${PROJECT_NAME}/EthercatBusBase.cpp
)


# https://stackoverflow.com/questions/2222162/how-to-apply-fvisibility-option-to-symbols-in-static-libraries
# special link option.
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--exclude-libs,ALL")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--exclude-libs,ALL")
target_link_options(${PROJECT_NAME} PRIVATE "-Wl,--exclude-libs,ALL")

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
    ${PROJECT_NAME} ${PACKAGE_DEPENDENCIES}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_include_directories(include)


#hide all symboles except the explicitly marked ones.
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties(${PROJECT_NAME} PROPERTIES C_VISIBILITY_PRESET hidden)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Target
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_package()
