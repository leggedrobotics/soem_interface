cmake_minimum_required(VERSION 2.8.12)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")
project(soem_rsl C)

find_package(catkin REQUIRED)

catkin_package(
   INCLUDE_DIRS 
        ./soem
        ./soem/osal
   LIBRARIES soem)

set(SOEM_INCLUDE_INSTALL_DIR include/soem)
set(SOEM_LIB_INSTALL_DIR lib)
set(BUILD_TESTS TRUE)

if(WIN32)
  set(OS "win32")
  include_directories(soem/oshw/win32/wpcap/Include)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    link_directories(${CMAKE_CURRENT_LIST_DIR}/soem/oshw/win32/wpcap/Lib/x64)
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    link_directories(${CMAKE_CURRENT_LIST_DIR}/soem/oshw/win32/wpcap/Lib)
  endif()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /D _CRT_SECURE_NO_WARNINGS")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  /WX")
  set(OS_LIBS wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX AND NOT APPLE)
  set(OS "linux")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
  set(OS_LIBS pthread rt)
elseif(APPLE)
  # This must come *before* linux or MacOSX will identify as Unix.
  set(OS "macosx")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
  set(OS_LIBS pthread pcap)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
  set(OS "rtk")
  message("ARCH is ${ARCH}")
  message("BSP is ${BSP}")
  include_directories(soem/oshw/${OS}/${ARCH})
  file(GLOB OSHW_EXTRA_SOURCES soem/oshw/${OS}/${ARCH}/*.c)
  set(OSHW_SOURCES "${OS_HW_SOURCES} ${OSHW_ARCHSOURCES}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-format")
  set(OS_LIBS "-Wl,--start-group -l${BSP} -l${ARCH} -lkern -ldev -lsio -lblock -lfs -lusb -llwip -leth -li2c -lrtc -lcan -lnand -lspi -lnor -lpwm -ladc -ltrace -lc -lm -Wl,--end-group")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rtems")
  message("Building for RTEMS")
  set(OS "rtems")
  set(SOEM_LIB_INSTALL_DIR ${LIB_DIR})
  set(BUILD_TESTS FALSE)
endif()

message("OS is ${OS}")

file(GLOB SOEM_SOURCES soem/soem/*.c)
file(GLOB OSAL_SOURCES soem/osal/${OS}/*.c)
file(GLOB OSHW_SOURCES soem/oshw/${OS}/*.c)

file(GLOB SOEM_HEADERS soem/soem/*.h)
file(GLOB OSAL_HEADERS soem/osal/osal.h soem/osal/${OS}/*.h)
file(GLOB OSHW_HEADERS soem/oshw/${OS}/*.h)

message(${OSAL_HEADERS})

add_library(soem STATIC
  ${SOEM_SOURCES}
  ${OSAL_SOURCES}
  ${OSHW_SOURCES}
  ${OSHW_EXTRA_SOURCES})
target_link_libraries(soem ${OS_LIBS})
set_target_properties (soem PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(soem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/soem/soem>
  $<INSTALL_INTERFACE:include/soem/soem>)

target_include_directories(soem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/soem/osal>
  $<INSTALL_INTERFACE:include/soem/soem>)

target_include_directories(soem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/soem/osal/${OS}>
  $<INSTALL_INTERFACE:include/soem/soem>)

target_include_directories(soem
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/soem/oshw/${OS}>
  $<INSTALL_INTERFACE:include/soem/soem>
  )

message("LIB_DIR: ${SOEM_LIB_INSTALL_DIR}")

install(TARGETS soem EXPORT soemConfig DESTINATION ${SOEM_LIB_INSTALL_DIR})

install(EXPORT soemConfig DESTINATION share/soem/cmake)

install(FILES
  ${SOEM_HEADERS}
  ${OSAL_HEADERS}
  ${OSHW_HEADERS}
  DESTINATION ${SOEM_INCLUDE_INSTALL_DIR})
  
add_custom_target(link_target ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_INSTALL_PREFIX}/include/soem/ethercattype.h ${CMAKE_INSTALL_PREFIX}/include/ethercattype.h)

if(BUILD_TESTS) 
  add_subdirectory(soem/test/linux/slaveinfo)
  add_subdirectory(soem/test/linux/eepromtool)
  add_subdirectory(soem/test/linux/simple_test)
endif()
