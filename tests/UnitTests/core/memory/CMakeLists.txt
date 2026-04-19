# Memory unit tests

add_executable(thread_safe_arena_test thread_safe_arena_test.cpp)
phynity_target_defaults(thread_safe_arena_test)
phynity_apply_warnings(thread_safe_arena_test)
phynity_apply_sanitizers(thread_safe_arena_test)
target_link_libraries(thread_safe_arena_test PRIVATE phynity::core Catch2::Catch2WithMain)
add_test(NAME unit.memory.thread_safe_arena COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:thread_safe_arena_test>)
