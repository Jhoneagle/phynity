add_library(phynity_core)
add_library(phynity::core ALIAS phynity_core)

target_include_directories(phynity_core PUBLIC
	${CMAKE_SOURCE_DIR}/src/core
)

# Job system sources
target_sources(phynity_core PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/jobs/job_system.cpp
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
)

target_link_libraries(phynity_core PUBLIC phynity::platform)

phynity_target_defaults(phynity_core)
phynity_apply_warnings(phynity_core)
phynity_apply_sanitizers(phynity_core)
