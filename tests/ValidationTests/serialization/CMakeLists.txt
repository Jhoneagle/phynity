# Serialization round-trip and golden file tests

add_executable(serialization_round_trip_test serialization_round_trip_test.cpp)
phynity_target_defaults(serialization_round_trip_test)
phynity_apply_warnings(serialization_round_trip_test)
phynity_apply_sanitizers(serialization_round_trip_test)
target_link_libraries(serialization_round_trip_test PRIVATE phynity::tests Catch2::Catch2WithMain)

add_executable(replay_round_trip_test replay_round_trip_test.cpp)
phynity_target_defaults(replay_round_trip_test)
phynity_apply_warnings(replay_round_trip_test)
phynity_apply_sanitizers(replay_round_trip_test)
target_link_libraries(replay_round_trip_test PRIVATE phynity::tests Catch2::Catch2WithMain)

# Compute absolute path for golden files directory
get_filename_component(GOLDEN_FILES_DIR_ABS "${CMAKE_CURRENT_SOURCE_DIR}/../../golden_outputs" ABSOLUTE)

target_compile_definitions(serialization_round_trip_test PRIVATE
	GOLDEN_FILES_DIR=${GOLDEN_FILES_DIR_ABS}
)

if(GOLDEN_CAPTURE_MODE)
	target_compile_definitions(serialization_round_trip_test PRIVATE GOLDEN_CAPTURE_MODE)
endif()

if(GOLDEN_CAPTURE_MODE)
	target_compile_definitions(replay_round_trip_test PRIVATE
		GOLDEN_FILES_DIR=${GOLDEN_FILES_DIR_ABS}
	)
	target_compile_definitions(replay_round_trip_test PRIVATE GOLDEN_CAPTURE_MODE)
endif()

add_test(NAME validation.serialization.round_trip
	COMMAND ${CMAKE_COMMAND} -E env
	GOLDEN_FILES_DIR="${GOLDEN_FILES_DIR_ABS}"
	$<TARGET_FILE:serialization_round_trip_test>
)

add_test(NAME validation.serialization.replay_round_trip
	COMMAND ${CMAKE_COMMAND} -E env
	GOLDEN_FILES_DIR="${GOLDEN_FILES_DIR_ABS}"
	$<TARGET_FILE:replay_round_trip_test>
)

set_tests_properties(validation.serialization.round_trip PROPERTIES LABELS "validation;serialization;golden")
set_tests_properties(validation.serialization.replay_round_trip PROPERTIES LABELS "validation;serialization")
