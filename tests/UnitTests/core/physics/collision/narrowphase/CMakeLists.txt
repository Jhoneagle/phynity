# Narrowphase collision unit tests

add_executable(aabb_narrowphase_test aabb_narrowphase_test.cpp)
phynity_target_defaults(aabb_narrowphase_test)
phynity_apply_warnings(aabb_narrowphase_test)
phynity_apply_sanitizers(aabb_narrowphase_test)
target_link_libraries(aabb_narrowphase_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.collision.aabb_narrowphase COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:aabb_narrowphase_test>)

add_executable(gjk_epa_test gjk_epa_test.cpp)
phynity_target_defaults(gjk_epa_test)
phynity_apply_warnings(gjk_epa_test)
phynity_apply_sanitizers(gjk_epa_test)
target_link_libraries(gjk_epa_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.collision.gjk_epa COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:gjk_epa_test>)

add_executable(sat_solver_test sat_solver_test.cpp)
phynity_target_defaults(sat_solver_test)
phynity_apply_warnings(sat_solver_test)
phynity_apply_sanitizers(sat_solver_test)
target_link_libraries(sat_solver_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.collision.sat_solver COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:sat_solver_test>)
