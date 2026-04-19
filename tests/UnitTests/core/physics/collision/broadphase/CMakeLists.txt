# Broadphase collision unit tests

add_executable(spatial_grid_hash_test spatial_grid_hash_test.cpp)
phynity_target_defaults(spatial_grid_hash_test)
phynity_apply_warnings(spatial_grid_hash_test)
phynity_apply_sanitizers(spatial_grid_hash_test)
target_link_libraries(spatial_grid_hash_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.collision.spatial_grid_hash COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:spatial_grid_hash_test>)

add_executable(spatial_grid_test spatial_grid_test.cpp)
phynity_target_defaults(spatial_grid_test)
phynity_apply_warnings(spatial_grid_test)
phynity_apply_sanitizers(spatial_grid_test)
target_link_libraries(spatial_grid_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.collision.spatial_grid COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:spatial_grid_test>)
