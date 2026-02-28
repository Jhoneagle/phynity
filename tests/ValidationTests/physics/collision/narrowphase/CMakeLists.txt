# Narrowphase collision tests

add_executable(sat_validation_test sat_validation_test.cpp)
phynity_target_defaults(sat_validation_test)
phynity_apply_warnings(sat_validation_test)
phynity_apply_sanitizers(sat_validation_test)
target_link_libraries(sat_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.collision.sat COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:sat_validation_test>)
set_tests_properties(validation.physics.collision.sat PROPERTIES LABELS validation)

add_executable(gjk_epa_validation_test gjk_epa_validation_test.cpp)
phynity_target_defaults(gjk_epa_validation_test)
phynity_apply_warnings(gjk_epa_validation_test)
phynity_apply_sanitizers(gjk_epa_validation_test)
target_link_libraries(gjk_epa_validation_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME validation.physics.collision.gjk_epa COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:gjk_epa_validation_test>)
set_tests_properties(validation.physics.collision.gjk_epa PROPERTIES LABELS validation)
