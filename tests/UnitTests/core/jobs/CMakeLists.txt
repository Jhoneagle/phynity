add_executable(job_system_test job_system_test.cpp)
target_link_libraries(job_system_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(job_system_test)

add_test(NAME unit.jobs.job_system COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:job_system_test>)
