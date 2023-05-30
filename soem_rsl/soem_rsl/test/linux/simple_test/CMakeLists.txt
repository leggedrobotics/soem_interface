
set(SOURCES simple_test.c)
add_executable(simple_test ${SOURCES})
target_link_libraries(simple_test soem_rsl)
install(TARGETS simple_test DESTINATION bin)
