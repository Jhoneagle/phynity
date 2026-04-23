# Serialization unit tests

add_executable(simulation_timeline_test simulation_timeline_test.cpp)
phynity_target_defaults(simulation_timeline_test)
phynity_apply_warnings(simulation_timeline_test)
phynity_apply_sanitizers(simulation_timeline_test)
target_link_libraries(simulation_timeline_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.serialization.simulation_timeline COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:simulation_timeline_test>)
