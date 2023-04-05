cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../../cmake/Modules")
project(soem_rsl)

set(soem_PROJECT_DIR
  ${PROJECT_SOURCE_DIR}/../..
)

if(DEFINED HOST_INSTALL)
  set(SOEM_INCLUDE_INSTALL_DIR include/soem_rsl/soem_rsl)
else()
  #set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
  set(SOEM_INCLUDE_INSTALL_DIR include)
endif()

#message("CMAKE_SYSTEM_NAME is ${CMAKE_SYSTEM_NAME}")

if(WIN32)
  set(OS "win32")
  include_directories(include_win32/soem_rsl/oshw/wpcap/Include)
  link_directories(${CMAKE_SOURCE_DIR}/include_win32/soem_rsl/oshw/wpcap/Lib)
elseif(UNIX)
  set(OS "linux")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
  set(OS "rtk")
  #   message("ARCH is ${ARCH}")
  #   message("BSP is ${BSP}")
  include_directories(include_${OS}/soem_rsl/oshw/${ARCH})
  add_definitions("-Wno-unused-but-set-variable")
  add_definitions("-Wno-unused-function")
  add_definitions("-Wno-format")
endif()

#message("OS is ${OS}")

set(soem_INCLUDE_DIRS
  ${soem_PROJECT_DIR}/include
  ${soem_PROJECT_DIR}/include_${OS}
)

set(soem_LIBRARIES
  soem_rsl
)

include_directories(
  ${soem_INCLUDE_DIRS}
)

if (MSVC)
  set(CMAKE_C_FLAGS_RELEASE "/D _CRT_SECURE_NO_WARNINGS /EHsc")
  set(CMAKE_C_FLAGS_DEBUG "/D _CRT_SECURE_NO_WARNINGS /EHsc /ZI /Od")
endif()

file(GLOB SOEM_SOURCES ${soem_PROJECT_DIR}/include/soem_rsl/soem_rsl/*.c)
file(GLOB OSAL_SOURCES ${soem_PROJECT_DIR}/include_${OS}/soem_rsl/osal/*.c)
file(GLOB OSHW_SOURCES ${soem_PROJECT_DIR}/include_${OS}/soem_rsl/oshw/*.c)

if(WIN32)
  set(SOEM_SOURCES ${SOEM_SOURCES})
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
  file(GLOB OSHW_ARCHSOURCES ${soem_PROJECT_DIR}/include_${OS}/soem_rsl/oshw/${ARCH}/*.c)
endif()

add_library(soem_rsl ${SOEM_SOURCES} ${OSAL_SOURCES} ${OSHW_SOURCES} ${OSHW_ARCHSOURCES})
if(WIN32)
  target_link_libraries(soem_rsl wpcap.lib Packet.lib Ws2_32.lib Winmm.lib ${HDF5_LIBRARIES})
elseif(UNIX)
  target_link_libraries(soem_rsl pthread rt ${catkin_LIBRARIES})
endif()

if(NOT ${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
  set(soem_INCLUDE_DIRS ${soem_INCLUDE_DIRS} PARENT_SCOPE)
  set(soem_LIBRARIES ${soem_LIBRARIES} PARENT_SCOPE)
endif()
