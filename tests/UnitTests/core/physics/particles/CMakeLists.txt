# Particle system unit tests

add_executable(particle_test particle_test.cpp)
phynity_target_defaults(particle_test)
phynity_apply_warnings(particle_test)
phynity_apply_sanitizers(particle_test)
target_link_libraries(particle_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.particle COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:particle_test>)

add_executable(particle_system_test particle_system_test.cpp)
phynity_target_defaults(particle_system_test)
phynity_apply_warnings(particle_system_test)
phynity_apply_sanitizers(particle_system_test)
target_link_libraries(particle_system_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.particle_system COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:particle_system_test>)
