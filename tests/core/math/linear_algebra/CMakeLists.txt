add_executable(lu_decomposition_test lu_decomposition_test.cpp)
phynity_target_defaults(lu_decomposition_test)
phynity_apply_warnings(lu_decomposition_test)
phynity_apply_sanitizers(lu_decomposition_test)
target_link_libraries(lu_decomposition_test PRIVATE phynity::core Catch2::Catch2WithMain)
add_test(NAME core.linear_algebra.lu COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:lu_decomposition_test>)

add_executable(qr_decomposition_test qr_decomposition_test.cpp)
phynity_target_defaults(qr_decomposition_test)
phynity_apply_warnings(qr_decomposition_test)
phynity_apply_sanitizers(qr_decomposition_test)
target_link_libraries(qr_decomposition_test PRIVATE phynity::core Catch2::Catch2WithMain)
add_test(NAME core.linear_algebra.qr COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:qr_decomposition_test>)

add_executable(linear_solver_test linear_solver_test.cpp)
phynity_target_defaults(linear_solver_test)
phynity_apply_warnings(linear_solver_test)
phynity_apply_sanitizers(linear_solver_test)
target_link_libraries(linear_solver_test PRIVATE phynity::core Catch2::Catch2WithMain)
add_test(NAME core.linear_algebra.solver COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:linear_solver_test>)


