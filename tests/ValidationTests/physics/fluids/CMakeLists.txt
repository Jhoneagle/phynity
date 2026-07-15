# Fluids (SPH/PBF) validation tests

add_executable(sph_fluid_validation_test sph_fluid_validation_test.cpp)
phynity_target_defaults(sph_fluid_validation_test)
phynity_apply_warnings(sph_fluid_validation_test)
phynity_apply_sanitizers(sph_fluid_validation_test)
target_link_libraries(sph_fluid_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.fluids.sph COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:sph_fluid_validation_test>)
set_tests_properties(validation.physics.fluids.sph PROPERTIES LABELS validation)

add_executable(pbf_fluid_validation_test pbf_fluid_validation_test.cpp)
phynity_target_defaults(pbf_fluid_validation_test)
phynity_apply_warnings(pbf_fluid_validation_test)
phynity_apply_sanitizers(pbf_fluid_validation_test)
target_link_libraries(pbf_fluid_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.fluids.pbf COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:pbf_fluid_validation_test>)
set_tests_properties(validation.physics.fluids.pbf PROPERTIES LABELS validation)
