# CCD unit tests

add_executable(ccd_swept_sphere_test ccd_swept_sphere_test.cpp)
phynity_target_defaults(ccd_swept_sphere_test)
phynity_apply_warnings(ccd_swept_sphere_test)
phynity_apply_sanitizers(ccd_swept_sphere_test)
target_link_libraries(ccd_swept_sphere_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.collision.ccd_swept_sphere COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:ccd_swept_sphere_test>)

add_executable(ccd_convex_sweep_test ccd_convex_sweep_test.cpp)
phynity_target_defaults(ccd_convex_sweep_test)
phynity_apply_warnings(ccd_convex_sweep_test)
phynity_apply_sanitizers(ccd_convex_sweep_test)
target_link_libraries(ccd_convex_sweep_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.collision.ccd_convex_sweep COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:ccd_convex_sweep_test>)
