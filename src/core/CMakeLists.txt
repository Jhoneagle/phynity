add_library(phynity_core)
add_library(phynity::core ALIAS phynity_core)
set_target_properties(phynity_core PROPERTIES EXPORT_NAME core)

target_include_directories(phynity_core PUBLIC
	$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/core>
	$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Job system sources
target_sources(phynity_core PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/jobs/job_system.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/jobs/task_graph.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/jobs/task_scheduler.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/jobs/task_executor.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/jobs/schedule_recorder.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/jobs/schedule_replayer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/physics/constraints/constraint_graph.cpp
)

# Physics: Continuous Collision Detection (CCD) sources
target_sources(phynity_core PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/physics/collision/ccd/swept_sphere.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/physics/collision/ccd/linear_cast.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/physics/collision/ccd/conservative_advancement.cpp
)

# Serialization sources
target_sources(phynity_core PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/serialization/snapshot_schema.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/serialization/snapshot_migration.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/serialization/snapshot_serializer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/serialization/snapshot_helpers.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/serialization/replay_writer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/serialization/replay_reader.cpp
)

target_link_libraries(phynity_core PUBLIC phynity::platform)

phynity_target_defaults(phynity_core)
phynity_apply_warnings(phynity_core)
phynity_apply_sanitizers(phynity_core)
