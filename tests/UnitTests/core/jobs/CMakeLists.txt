add_executable(job_system_test job_system_test.cpp)
target_link_libraries(job_system_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(job_system_test)

add_test(NAME unit.jobs.job_system COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:job_system_test>)

add_executable(task_graph_test task_graph_test.cpp)
target_link_libraries(task_graph_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(task_graph_test)

add_test(NAME unit.jobs.task_graph COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:task_graph_test>)

add_executable(task_scheduler_test task_scheduler_test.cpp)
target_link_libraries(task_scheduler_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(task_scheduler_test)

add_test(NAME unit.jobs.task_scheduler COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:task_scheduler_test>)

add_executable(task_executor_test task_executor_test.cpp)
target_link_libraries(task_executor_test PRIVATE
    phynity::core
    Catch2::Catch2WithMain
)
phynity_target_defaults(task_executor_test)

add_test(NAME unit.jobs.task_executor COMMAND ${CMAKE_COMMAND} -E env $<TARGET_FILE:task_executor_test>)

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
