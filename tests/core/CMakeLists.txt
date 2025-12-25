# Core unit tests without external frameworks.

add_executable(vec3_test vec3_test.cpp)
phynity_target_defaults(vec3_test)
phynity_apply_warnings(vec3_test)
phynity_apply_sanitizers(vec3_test)
target_link_libraries(vec3_test PRIVATE phynity::core)
add_test(NAME core.vec3 COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:vec3_test>)

add_executable(particle_test particle_test.cpp)
phynity_target_defaults(particle_test)
phynity_apply_warnings(particle_test)
phynity_apply_sanitizers(particle_test)
target_link_libraries(particle_test PRIVATE phynity::core)
add_test(NAME core.particle COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:particle_test>)
