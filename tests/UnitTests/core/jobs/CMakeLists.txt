add_executable(test_job_system test_job_system.cpp)
target_link_libraries(test_job_system PRIVATE 
    phynity::core
    Catch2::Catch2WithMain
)

add_test(NAME job_system_tests COMMAND test_job_system)
