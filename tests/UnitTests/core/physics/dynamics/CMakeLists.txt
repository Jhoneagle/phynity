# Dynamics unit tests

add_executable(force_field_test force_field_test.cpp)
phynity_target_defaults(force_field_test)
phynity_apply_warnings(force_field_test)
phynity_apply_sanitizers(force_field_test)
target_link_libraries(force_field_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.force_field COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:force_field_test>)

add_executable(material_test material_test.cpp)
phynity_target_defaults(material_test)
phynity_apply_warnings(material_test)
phynity_apply_sanitizers(material_test)
target_link_libraries(material_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.material COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:material_test>)

add_executable(timestep_controller_test timestep_controller_test.cpp)
phynity_target_defaults(timestep_controller_test)
phynity_apply_warnings(timestep_controller_test)
phynity_apply_sanitizers(timestep_controller_test)
target_link_libraries(timestep_controller_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.timestep_controller COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:timestep_controller_test>)
