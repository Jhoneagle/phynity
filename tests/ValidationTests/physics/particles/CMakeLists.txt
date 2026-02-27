# Particle system validation tests

add_executable(physics_validation_test physics_validation_test.cpp)
phynity_target_defaults(physics_validation_test)
phynity_apply_warnings(physics_validation_test)
phynity_apply_sanitizers(physics_validation_test)
target_link_libraries(physics_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.system COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:physics_validation_test>)

# Golden test executable for particle trajectories
add_executable(particle_golden_test particle_golden_test.cpp)
phynity_target_defaults(particle_golden_test)
phynity_apply_warnings(particle_golden_test)
phynity_apply_sanitizers(particle_golden_test)
target_link_libraries(particle_golden_test PRIVATE phynity::tests Catch2::Catch2WithMain)

add_executable(integration_scenes_golden_test integration_scenes_golden_test.cpp)
phynity_target_defaults(integration_scenes_golden_test)
phynity_apply_warnings(integration_scenes_golden_test)
phynity_apply_sanitizers(integration_scenes_golden_test)
target_link_libraries(integration_scenes_golden_test PRIVATE phynity::tests Catch2::Catch2WithMain)

# Compute absolute path for golden files directory
get_filename_component(GOLDEN_FILES_DIR_ABS "${CMAKE_CURRENT_SOURCE_DIR}/../../../golden_outputs" ABSOLUTE)

# Support golden capture mode (GOLDEN_CAPTURE_MODE=ON to regenerate goldens)
if(GOLDEN_CAPTURE_MODE)
	target_compile_definitions(particle_golden_test PRIVATE GOLDEN_CAPTURE_MODE)
	target_compile_definitions(integration_scenes_golden_test PRIVATE GOLDEN_CAPTURE_MODE)
endif()

# Set golden files directory for test to find/write files
target_compile_definitions(particle_golden_test PRIVATE
	GOLDEN_FILES_DIR=${GOLDEN_FILES_DIR_ABS}
)

target_compile_definitions(integration_scenes_golden_test PRIVATE
	GOLDEN_FILES_DIR=${GOLDEN_FILES_DIR_ABS}
)

add_test(NAME validation.physics.particle_golden
	COMMAND ${CMAKE_COMMAND} -E env
	GOLDEN_FILES_DIR="${GOLDEN_FILES_DIR_ABS}"
	$<TARGET_FILE:particle_golden_test>
)

add_test(NAME validation.physics.integration_scenes_golden
	COMMAND ${CMAKE_COMMAND} -E env
	GOLDEN_FILES_DIR="${GOLDEN_FILES_DIR_ABS}"
	$<TARGET_FILE:integration_scenes_golden_test>
)

# Tag golden tests for easy filtering
set_tests_properties(validation.physics.particle_golden PROPERTIES LABELS golden)
set_tests_properties(validation.physics.integration_scenes_golden PROPERTIES LABELS golden)
