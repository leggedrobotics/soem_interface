cmake_minimum_required(VERSION 3.10)

find_package(catkin REQUIRED)

project(soem_rsl C)

catkin_package(
    INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}
    LIBRARIES
        soem
)

add_subdirectory(soem)

set_target_properties(soem PROPERTIES POSITION_INDEPENDENT_CODE ON)
