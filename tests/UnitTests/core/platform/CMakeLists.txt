# Platform unit tests

add_executable(allocator_delta_test allocator_delta_test.cpp)
phynity_target_defaults(allocator_delta_test)
phynity_apply_warnings(allocator_delta_test)
phynity_apply_sanitizers(allocator_delta_test)
target_link_libraries(allocator_delta_test PRIVATE phynity::platform Catch2::Catch2WithMain)
add_test(NAME unit.platform.allocator_delta COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:allocator_delta_test>)
