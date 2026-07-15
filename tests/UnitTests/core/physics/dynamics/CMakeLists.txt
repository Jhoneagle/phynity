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

add_executable(coulomb_interaction_test coulomb_interaction_test.cpp)
phynity_target_defaults(coulomb_interaction_test)
phynity_apply_warnings(coulomb_interaction_test)
phynity_apply_sanitizers(coulomb_interaction_test)
target_link_libraries(coulomb_interaction_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.coulomb_interaction COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:coulomb_interaction_test>)
