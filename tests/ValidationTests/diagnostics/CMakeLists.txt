# Diagnostics validation tests

add_executable(profiler_overhead_benchmark profiler_overhead_benchmark.cpp)
phynity_target_defaults(profiler_overhead_benchmark)
phynity_apply_warnings(profiler_overhead_benchmark)
phynity_apply_sanitizers(profiler_overhead_benchmark)
target_link_libraries(profiler_overhead_benchmark PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.diagnostics.profiler_overhead COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:profiler_overhead_benchmark>)


# Diagnostics golden tests
add_executable(diagnostics_golden_test diagnostics_golden_test.cpp)
phynity_target_defaults(diagnostics_golden_test)
phynity_apply_warnings(diagnostics_golden_test)
phynity_apply_sanitizers(diagnostics_golden_test)
target_link_libraries(diagnostics_golden_test PRIVATE phynity::tests Catch2::Catch2WithMain)

# Compute absolute path for golden files directory
get_filename_component(GOLDEN_FILES_DIR_ABS "${CMAKE_CURRENT_SOURCE_DIR}/../../golden_outputs" ABSOLUTE)

if(GOLDEN_CAPTURE_MODE)
	target_compile_definitions(diagnostics_golden_test PRIVATE GOLDEN_CAPTURE_MODE)
endif()

target_compile_definitions(diagnostics_golden_test PRIVATE
	GOLDEN_FILES_DIR=${GOLDEN_FILES_DIR_ABS}
)

add_test(NAME validation.diagnostics.golden
	COMMAND ${CMAKE_COMMAND} -E env
	GOLDEN_FILES_DIR="${GOLDEN_FILES_DIR_ABS}"
	$<TARGET_FILE:diagnostics_golden_test>
)

set_tests_properties(validation.diagnostics.golden PROPERTIES LABELS golden)
