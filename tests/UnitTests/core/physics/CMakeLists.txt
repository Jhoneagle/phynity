# Core physics unit tests without external frameworks.

add_executable(particle_test particle_test.cpp)
phynity_target_defaults(particle_test)
phynity_apply_warnings(particle_test)
phynity_apply_sanitizers(particle_test)
target_link_libraries(particle_test PRIVATE phynity::tests)
add_test(NAME unit.physics.particle COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:particle_test>)
