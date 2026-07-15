# Dynamics (force field) validation tests

add_executable(forces_validation_test forces_validation_test.cpp)
phynity_target_defaults(forces_validation_test)
phynity_apply_warnings(forces_validation_test)
phynity_apply_sanitizers(forces_validation_test)
target_link_libraries(forces_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.dynamics.forces COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:forces_validation_test>)
set_tests_properties(validation.physics.dynamics.forces PROPERTIES LABELS validation)

# Electromagnetic force-field validation (cyclotron, E acceleration, E×B drift)
add_executable(em_forces_validation_test em_forces_validation_test.cpp)
phynity_target_defaults(em_forces_validation_test)
phynity_apply_warnings(em_forces_validation_test)
phynity_apply_sanitizers(em_forces_validation_test)
target_link_libraries(em_forces_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.dynamics.em COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:em_forces_validation_test>)
set_tests_properties(validation.physics.dynamics.em PROPERTIES LABELS validation)

# Force-field integration against the app-level PhysicsContext (single-source-of-truth
# gravity plumbing). Needs the sandbox context, so it is gated on that target.
if(TARGET sandbox_context)
    add_executable(forces_context_integration_test forces_context_integration_test.cpp)
    phynity_target_defaults(forces_context_integration_test)
    phynity_apply_warnings(forces_context_integration_test)
    phynity_apply_sanitizers(forces_context_integration_test)
    target_link_libraries(forces_context_integration_test PRIVATE phynity::tests sandbox_context Catch2::Catch2WithMain)
    add_test(NAME validation.physics.dynamics.forces_context COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:forces_context_integration_test>)
    set_tests_properties(validation.physics.dynamics.forces_context PROPERTIES LABELS validation)
endif()
