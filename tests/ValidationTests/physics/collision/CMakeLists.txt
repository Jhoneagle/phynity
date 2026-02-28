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

# PGS solver golden tests
add_executable(pgs_solver_golden_test pgs_solver_golden_test.cpp)
phynity_target_defaults(pgs_solver_golden_test)
phynity_apply_warnings(pgs_solver_golden_test)
phynity_apply_sanitizers(pgs_solver_golden_test)
target_link_libraries(pgs_solver_golden_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.collision.pgs_solver_golden COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:pgs_solver_golden_test>)
set_tests_properties(validation.physics.collision.pgs_solver_golden PROPERTIES LABELS golden)

# SAT validation tests
add_executable(sat_validation_test sat_validation_test.cpp)
phynity_target_defaults(sat_validation_test)
phynity_apply_warnings(sat_validation_test)
phynity_apply_sanitizers(sat_validation_test)
target_link_libraries(sat_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.collision.sat COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:sat_validation_test>)
set_tests_properties(validation.physics.collision.sat PROPERTIES LABELS validation)

# GJK/EPA validation tests
add_executable(gjk_epa_validation_test gjk_epa_validation_test.cpp)
phynity_target_defaults(gjk_epa_validation_test)
phynity_apply_warnings(gjk_epa_validation_test)
phynity_apply_sanitizers(gjk_epa_validation_test)
target_link_libraries(gjk_epa_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.collision.gjk_epa COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:gjk_epa_validation_test>)
set_tests_properties(validation.physics.collision.gjk_epa PROPERTIES LABELS validation)

add_executable(stacking_scenarios_test stacking_scenarios_test.cpp)
phynity_target_defaults(stacking_scenarios_test)
phynity_apply_warnings(stacking_scenarios_test)
phynity_apply_sanitizers(stacking_scenarios_test)
target_link_libraries(stacking_scenarios_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.collision.stacking_scenarios COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:stacking_scenarios_test>)
set_tests_properties(validation.physics.collision.stacking_scenarios PROPERTIES LABELS validation)

# CCD (Continuous Collision Detection) validation tests
add_executable(ccd_validation_test ccd_validation_test.cpp)
phynity_target_defaults(ccd_validation_test)
phynity_apply_warnings(ccd_validation_test)
phynity_apply_sanitizers(ccd_validation_test)
target_link_libraries(ccd_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.collision.ccd COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:ccd_validation_test>)
set_tests_properties(validation.physics.collision.ccd PROPERTIES LABELS validation)

