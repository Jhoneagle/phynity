# Sandbox context library (application-level physics wrapper)
add_library(sandbox_context OBJECT
	physics_context.cpp
)

target_include_directories(sandbox_context PUBLIC
	${CMAKE_SOURCE_DIR}/src
	${CMAKE_SOURCE_DIR}/sandbox
)

target_link_libraries(sandbox_context PUBLIC
	phynity::core
	phynity::platform
	phynity::render
)

phynity_target_defaults(sandbox_context)
phynity_apply_warnings(sandbox_context)
phynity_apply_sanitizers(sandbox_context)

# Demo scenarios library
add_library(sandbox_scenarios OBJECT
	demo_scenarios.cpp
)

target_include_directories(sandbox_scenarios PUBLIC
	${CMAKE_SOURCE_DIR}/src
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(sandbox_scenarios PUBLIC
	phynity::core
	sandbox_context
)

phynity_target_defaults(sandbox_scenarios)
phynity_apply_warnings(sandbox_scenarios)
phynity_apply_sanitizers(sandbox_scenarios)

# Sandbox executable
add_executable(phynity_sandbox
	main.cpp
	$<TARGET_OBJECTS:sandbox_context>
	$<TARGET_OBJECTS:sandbox_scenarios>
)
target_include_directories(phynity_sandbox PRIVATE
	${CMAKE_SOURCE_DIR}/src
	${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(phynity_sandbox PRIVATE
	phynity::core
	phynity::platform
	phynity::render
)
phynity_target_defaults(phynity_sandbox)
phynity_apply_warnings(phynity_sandbox)
phynity_apply_sanitizers(phynity_sandbox)
