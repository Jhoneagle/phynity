# Collision validation tests

add_executable(broadphase_correctness_validation_test broadphase_correctness_validation_test.cpp)
phynity_target_defaults(broadphase_correctness_validation_test)
phynity_apply_warnings(broadphase_correctness_validation_test)
phynity_apply_sanitizers(broadphase_correctness_validation_test)
target_link_libraries(broadphase_correctness_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.collision.broadphase_correctness COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:broadphase_correctness_validation_test>)

add_executable(broadphase_performance_benchmark_test broadphase_performance_benchmark_test.cpp)
phynity_target_defaults(broadphase_performance_benchmark_test)
phynity_apply_warnings(broadphase_performance_benchmark_test)
phynity_apply_sanitizers(broadphase_performance_benchmark_test)
target_link_libraries(broadphase_performance_benchmark_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.collision.broadphase_performance COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:broadphase_performance_benchmark_test>)

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

# Contact cache golden tests
add_executable(contact_cache_golden_test contact_cache_golden_test.cpp)
phynity_target_defaults(contact_cache_golden_test)
phynity_apply_warnings(contact_cache_golden_test)
phynity_apply_sanitizers(contact_cache_golden_test)
target_link_libraries(contact_cache_golden_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.collision.contact_cache_golden COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:contact_cache_golden_test>)
set_tests_properties(validation.physics.collision.contact_cache_golden PROPERTIES LABELS golden)
