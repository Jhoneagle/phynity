# Collision validation tests

add_subdirectory(broadphase)
add_subdirectory(narrowphase)
add_subdirectory(contact)

# Collision golden tests
add_executable(collision_golden_test collision_golden_test.cpp)
phynity_target_defaults(collision_golden_test)
phynity_apply_warnings(collision_golden_test)
phynity_apply_sanitizers(collision_golden_test)
target_link_libraries(collision_golden_test PRIVATE phynity::tests Catch2::Catch2WithMain)

# Compute absolute path for golden files directory
get_filename_component(GOLDEN_FILES_DIR_ABS "${CMAKE_CURRENT_SOURCE_DIR}/../../../golden_outputs" ABSOLUTE)

if(GOLDEN_CAPTURE_MODE)
	target_compile_definitions(collision_golden_test PRIVATE GOLDEN_CAPTURE_MODE)
endif()

target_compile_definitions(collision_golden_test PRIVATE
	GOLDEN_FILES_DIR=${GOLDEN_FILES_DIR_ABS}
)

add_test(NAME validation.physics.collision.golden
	COMMAND ${CMAKE_COMMAND} -E env
	GOLDEN_FILES_DIR="${GOLDEN_FILES_DIR_ABS}"
	$<TARGET_FILE:collision_golden_test>
)

set_tests_properties(validation.physics.collision.golden PROPERTIES LABELS golden)

# CCD (Continuous Collision Detection) validation tests
add_executable(ccd_validation_test ccd_validation_test.cpp)
phynity_target_defaults(ccd_validation_test)
phynity_apply_warnings(ccd_validation_test)
phynity_apply_sanitizers(ccd_validation_test)
target_link_libraries(ccd_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.collision.ccd COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:ccd_validation_test>)
set_tests_properties(validation.physics.collision.ccd PROPERTIES LABELS validation)

