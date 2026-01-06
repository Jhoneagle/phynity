# Utilities tests

add_executable(utilities_test utilities_test.cpp)
phynity_target_defaults(utilities_test)
phynity_apply_warnings(utilities_test)
phynity_apply_sanitizers(utilities_test)
target_link_libraries(utilities_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.utilities COMMAND utilities_test)