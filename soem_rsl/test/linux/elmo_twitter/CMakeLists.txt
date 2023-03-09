
set(SOURCES elmo_twitter.c ecatcomm.c ecatcomm_data.c)
add_executable(elmo_twitter ${SOURCES})
target_link_libraries(elmo_twitter soem)

if(WIN32)
   target_link_libraries(elmo_twitter wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX)
   target_link_libraries(elmo_twitter pthread rt)
endif()

if(NOT DEFINED HOST_INSTALL)
   install(TARGETS elmo_twitter DESTINATION bin)
endif()
