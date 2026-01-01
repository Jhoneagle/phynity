# Quaternion tests

add_executable(quat_test quat_test.cpp)
phynity_target_defaults(quat_test)
phynity_apply_warnings(quat_test)
phynity_apply_sanitizers(quat_test)
target_link_libraries(quat_test PRIVATE phynity::core Catch2::Catch2WithMain)
add_test(NAME core.quaternions.quat COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:quat_test>)
