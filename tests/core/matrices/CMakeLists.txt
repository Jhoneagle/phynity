# Matrix tests

add_executable(mat2_test mat2_test.cpp)
phynity_target_defaults(mat2_test)
phynity_apply_warnings(mat2_test)
phynity_apply_sanitizers(mat2_test)
target_link_libraries(mat2_test PRIVATE phynity::core Catch2::Catch2WithMain)
add_test(NAME core.matrices.mat2 COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:mat2_test>)

add_executable(mat3_test mat3_test.cpp)
phynity_target_defaults(mat3_test)
phynity_apply_warnings(mat3_test)
phynity_apply_sanitizers(mat3_test)
target_link_libraries(mat3_test PRIVATE phynity::core Catch2::Catch2WithMain)
add_test(NAME core.matrices.mat3 COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:mat3_test>)

add_executable(mat4_test mat4_test.cpp)
phynity_target_defaults(mat4_test)
phynity_apply_warnings(mat4_test)
phynity_apply_sanitizers(mat4_test)
target_link_libraries(mat4_test PRIVATE phynity::core Catch2::Catch2WithMain)
add_test(NAME core.matrices.mat4 COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:mat4_test>)
