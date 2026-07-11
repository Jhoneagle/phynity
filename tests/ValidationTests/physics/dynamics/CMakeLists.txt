# Dynamics (force field) validation tests

add_executable(forces_validation_test forces_validation_test.cpp)
phynity_target_defaults(forces_validation_test)
phynity_apply_warnings(forces_validation_test)
phynity_apply_sanitizers(forces_validation_test)
target_link_libraries(forces_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.dynamics.forces COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:forces_validation_test>)
set_tests_properties(validation.physics.dynamics.forces PROPERTIES LABELS validation)
