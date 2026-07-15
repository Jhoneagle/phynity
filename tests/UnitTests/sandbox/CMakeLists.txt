# Sandbox utility unit tests

# The field-line tracer is a self-contained sandbox visualization header, so the
# test only needs the sandbox include directory (for the header) plus core math.
add_executable(field_line_tracer_test field_line_tracer_test.cpp)
phynity_target_defaults(field_line_tracer_test)
phynity_apply_warnings(field_line_tracer_test)
phynity_apply_sanitizers(field_line_tracer_test)
target_include_directories(field_line_tracer_test PRIVATE ${CMAKE_SOURCE_DIR}/sandbox)
target_link_libraries(field_line_tracer_test PRIVATE phynity::tests Catch2::Catch2WithMain)
add_test(NAME unit.sandbox.field_line_tracer COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:field_line_tracer_test>)
