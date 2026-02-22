# Validation/Integration tests (slower, cross-component)

add_executable(integration_test_harness_test integration_test_harness_test.cpp)
phynity_target_defaults(integration_test_harness_test)
phynity_apply_warnings(integration_test_harness_test)
phynity_apply_sanitizers(integration_test_harness_test)
target_link_libraries(integration_test_harness_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.calculus.integration_test_harness COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:integration_test_harness_test>)

# Physics system validation tests
add_executable(physics_validation_test physics_validation_test.cpp)
phynity_target_defaults(physics_validation_test)
phynity_apply_warnings(physics_validation_test)
phynity_apply_sanitizers(physics_validation_test)
target_link_libraries(physics_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.system COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:physics_validation_test>)

