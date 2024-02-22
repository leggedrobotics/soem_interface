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
  message_logger
  soem_rsl
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

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

add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# https://stackoverflow.com/questions/2222162/how-to-apply-fvisibility-option-to-symbols-in-static-libraries
# special link option.
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--exclude-libs,ALL")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--exclude-libs,ALL")
target_link_options(${PROJECT_NAME} PRIVATE "-Wl,--exclude-libs,ALL")

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#hide all symboles except the explicitly marked ones.
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties(${PROJECT_NAME} PROPERTIES C_VISIBILITY_PRESET hidden)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
