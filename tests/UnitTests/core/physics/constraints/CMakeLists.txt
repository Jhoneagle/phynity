# Constraint system unit tests

add_executable(constraints_test constraints_test.cpp)
phynity_target_defaults(constraints_test)
phynity_apply_warnings(constraints_test)
phynity_apply_sanitizers(constraints_test)
target_link_libraries(constraints_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.constraints COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:constraints_test>)

add_executable(constraint_jacobian_test constraint_jacobian_test.cpp)
phynity_target_defaults(constraint_jacobian_test)
phynity_apply_warnings(constraint_jacobian_test)
phynity_apply_sanitizers(constraint_jacobian_test)
target_link_libraries(constraint_jacobian_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.constraint_jacobian COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:constraint_jacobian_test>)
