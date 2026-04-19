# Contact resolution unit tests

add_executable(contact_cache_test contact_cache_test.cpp)
phynity_target_defaults(contact_cache_test)
phynity_apply_warnings(contact_cache_test)
phynity_apply_sanitizers(contact_cache_test)
target_link_libraries(contact_cache_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.physics.collision.contact_cache COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:contact_cache_test>)
