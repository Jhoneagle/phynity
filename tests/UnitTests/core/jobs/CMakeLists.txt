add_executable(test_job_system test_job_system.cpp)
target_link_libraries(test_job_system PRIVATE 
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(test_job_system)

add_test(NAME unit.jobs.job_system COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:test_job_system>)
