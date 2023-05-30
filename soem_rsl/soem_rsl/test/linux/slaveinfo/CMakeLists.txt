
set(SOURCES slaveinfo.c)
add_executable(slaveinfo ${SOURCES})
target_link_libraries(slaveinfo soem_rsl)
install(TARGETS slaveinfo DESTINATION bin)
