add_executable(job_system_test job_system_test.cpp)
target_link_libraries(job_system_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(job_system_test)

add_test(NAME unit.jobs.job_system COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:job_system_test>)


add_executable(work_stealing_deque_test work_stealing_deque_test.cpp)
target_link_libraries(work_stealing_deque_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(work_stealing_deque_test)

add_test(NAME unit.jobs.work_stealing_deque COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:work_stealing_deque_test>)

add_executable(thread_affinity_test thread_affinity_test.cpp)
target_link_libraries(thread_affinity_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(thread_affinity_test)

add_test(NAME unit.platform.thread_affinity COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:thread_affinity_test>)

add_executable(schedule_replay_test schedule_replay_test.cpp)
target_link_libraries(schedule_replay_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(schedule_replay_test)

add_test(NAME unit.jobs.schedule_replay COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:schedule_replay_test>)

add_executable(job_id_test job_id_test.cpp)
target_link_libraries(job_id_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(job_id_test)

add_test(NAME unit.jobs.job_id COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:job_id_test>)

add_executable(eventcount_test eventcount_test.cpp)
target_link_libraries(eventcount_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(eventcount_test)

add_test(NAME unit.jobs.eventcount COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:eventcount_test>)

add_executable(counter_test counter_test.cpp)
target_link_libraries(counter_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(counter_test)

add_test(NAME unit.jobs.counter COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:counter_test>)

add_executable(job_pool_test job_pool_test.cpp)
target_link_libraries(job_pool_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(job_pool_test)

add_test(NAME unit.jobs.job_pool COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:job_pool_test>)

add_executable(job_graph_test job_graph_test.cpp)
target_link_libraries(job_graph_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(job_graph_test)

add_test(NAME unit.jobs.job_graph COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:job_graph_test>)
