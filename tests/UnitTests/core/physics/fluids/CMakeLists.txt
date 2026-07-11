# Fluids (SPH/PBF) unit tests

add_executable(sph_kernels_test sph_kernels_test.cpp)
phynity_target_defaults(sph_kernels_test)
phynity_apply_warnings(sph_kernels_test)
phynity_apply_sanitizers(sph_kernels_test)
target_link_libraries(sph_kernels_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.sph_kernels COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:sph_kernels_test>)

add_executable(fluid_state_test fluid_state_test.cpp)
phynity_target_defaults(fluid_state_test)
phynity_apply_warnings(fluid_state_test)
phynity_apply_sanitizers(fluid_state_test)
target_link_libraries(fluid_state_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.fluid_state COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:fluid_state_test>)

add_executable(fluid_neighbor_search_test fluid_neighbor_search_test.cpp)
phynity_target_defaults(fluid_neighbor_search_test)
phynity_apply_warnings(fluid_neighbor_search_test)
phynity_apply_sanitizers(fluid_neighbor_search_test)
target_link_libraries(fluid_neighbor_search_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.fluid_neighbor_search COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:fluid_neighbor_search_test>)
