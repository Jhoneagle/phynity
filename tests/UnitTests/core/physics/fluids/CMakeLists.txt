# Fluids (SPH/PBF) unit tests

add_executable(sph_kernels_test sph_kernels_test.cpp)
phynity_target_defaults(sph_kernels_test)
phynity_apply_warnings(sph_kernels_test)
phynity_apply_sanitizers(sph_kernels_test)
target_link_libraries(sph_kernels_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.sph_kernels COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:sph_kernels_test>)
