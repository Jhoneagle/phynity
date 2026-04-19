# Collision system unit tests

add_subdirectory(broadphase)
add_subdirectory(narrowphase)
add_subdirectory(contact)
add_subdirectory(ccd)

add_executable(aabb_test aabb_test.cpp)
phynity_target_defaults(aabb_test)
phynity_apply_warnings(aabb_test)
phynity_apply_sanitizers(aabb_test)
target_link_libraries(aabb_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.collision.aabb COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:aabb_test>)

add_executable(convex_hull_test convex_hull_test.cpp)
phynity_target_defaults(convex_hull_test)
phynity_apply_warnings(convex_hull_test)
phynity_apply_sanitizers(convex_hull_test)
target_link_libraries(convex_hull_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.collision.convex_hull COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:convex_hull_test>)
