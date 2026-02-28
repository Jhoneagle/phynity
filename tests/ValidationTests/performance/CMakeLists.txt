# Performance regression tests

add_executable(collision_regression_test collision_regression_test.cpp)
phynity_target_defaults(collision_regression_test)
phynity_apply_warnings(collision_regression_test)
phynity_apply_sanitizers(collision_regression_test)
target_link_libraries(collision_regression_test PRIVATE phynity::tests Catch2::Catch2WithMain)

# Compute absolute path for golden files directory
get_filename_component(GOLDEN_FILES_DIR_ABS "${CMAKE_CURRENT_SOURCE_DIR}/../../golden_outputs" ABSOLUTE)

target_compile_definitions(collision_regression_test PRIVATE
	GOLDEN_FILES_DIR=${GOLDEN_FILES_DIR_ABS}
)

if(GOLDEN_CAPTURE_MODE)
	target_compile_definitions(collision_regression_test PRIVATE GOLDEN_CAPTURE_MODE)
endif()

add_test(NAME validation.performance.collision_regression
	COMMAND ${CMAKE_COMMAND} -E env
	GOLDEN_FILES_DIR="${GOLDEN_FILES_DIR_ABS}"
	$<TARGET_FILE:collision_regression_test>
)

set_tests_properties(validation.performance.collision_regression PROPERTIES LABELS validation)
