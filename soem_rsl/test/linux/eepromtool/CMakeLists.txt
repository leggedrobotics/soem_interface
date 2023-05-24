
set(SOURCES eepromtool.c)
add_executable(eepromtool ${SOURCES})
target_link_libraries(eepromtool soem_rsl)
install(TARGETS eepromtool DESTINATION bin)
