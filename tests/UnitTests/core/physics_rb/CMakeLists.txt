# Rigid body physics unit tests

add_executable(rigid_body_test rigid_body_tests.cpp)
phynity_target_defaults(rigid_body_test)
phynity_apply_warnings(rigid_body_test)
phynity_apply_sanitizers(rigid_body_test)
target_link_libraries(rigid_body_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.rigid_body COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:rigid_body_test>)
