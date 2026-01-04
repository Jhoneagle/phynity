# Quaternion tests

add_executable(quat_test quat_test.cpp)
phynity_target_defaults(quat_test)
phynity_apply_warnings(quat_test)
phynity_apply_sanitizers(quat_test)
target_link_libraries(quat_test PRIVATE phynity::core Catch2::Catch2WithMain)
add_test(NAME core.quaternions.quat COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:quat_test>)

add_executable(quat_conversions_test quat_conversions_test.cpp)
phynity_target_defaults(quat_conversions_test)
phynity_apply_warnings(quat_conversions_test)
phynity_apply_sanitizers(quat_conversions_test)
target_link_libraries(quat_conversions_test PRIVATE phynity::core Catch2::Catch2WithMain)
add_test(NAME core.quaternions.conversions COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:quat_conversions_test>)

add_executable(quat_interpolation_test quat_interpolation_test.cpp)
phynity_target_defaults(quat_interpolation_test)
phynity_apply_warnings(quat_interpolation_test)
phynity_apply_sanitizers(quat_interpolation_test)
target_link_libraries(quat_interpolation_test PRIVATE phynity::core Catch2::Catch2WithMain)
add_test(NAME core.quaternions.interpolation COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:quat_interpolation_test>)
