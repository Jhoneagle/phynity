add_subdirectory(calculus)

# Math golden tests
add_executable(math_golden_test math_golden_test.cpp)
phynity_target_defaults(math_golden_test)
phynity_apply_warnings(math_golden_test)
phynity_apply_sanitizers(math_golden_test)
target_link_libraries(math_golden_test PRIVATE phynity::tests Catch2::Catch2WithMain)

# Compute absolute path for golden files directory
get_filename_component(GOLDEN_FILES_DIR_ABS "${CMAKE_CURRENT_SOURCE_DIR}/../../golden_outputs" ABSOLUTE)

if(GOLDEN_CAPTURE_MODE)
	target_compile_definitions(math_golden_test PRIVATE GOLDEN_CAPTURE_MODE)
endif()

target_compile_definitions(math_golden_test PRIVATE
	GOLDEN_FILES_DIR=${GOLDEN_FILES_DIR_ABS}
)

add_test(NAME validation.math.golden
	COMMAND ${CMAKE_COMMAND} -E env
	GOLDEN_FILES_DIR="${GOLDEN_FILES_DIR_ABS}"
	$<TARGET_FILE:math_golden_test>
)

set_tests_properties(validation.math.golden PROPERTIES LABELS golden)
