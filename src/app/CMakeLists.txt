add_library(phynity_app OBJECT
	physics_context.cpp
)
add_library(phynity::app ALIAS phynity_app)

target_include_directories(phynity_app PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(phynity_app PUBLIC
	phynity::core
	phynity::platform
	phynity::render
)

phynity_target_defaults(phynity_app)
phynity_apply_warnings(phynity_app)
phynity_apply_sanitizers(phynity_app)

# Demo scenarios library
add_library(phynity_scenarios OBJECT
	demo_scenarios.cpp
)
add_library(phynity::scenarios ALIAS phynity_scenarios)

target_include_directories(phynity_scenarios PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(phynity_scenarios PUBLIC
	phynity::core
	phynity::app
)

phynity_target_defaults(phynity_scenarios)
phynity_apply_warnings(phynity_scenarios)
phynity_apply_sanitizers(phynity_scenarios)

# Executable
add_executable(phynity_demo 
	main.cpp
	$<TARGET_OBJECTS:phynity_app>
	$<TARGET_OBJECTS:phynity_scenarios>
)
target_include_directories(phynity_demo PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/..
)
target_link_libraries(phynity_demo PRIVATE 
	phynity::core
	phynity::platform
	phynity::render
)
phynity_target_defaults(phynity_demo)
phynity_apply_warnings(phynity_demo)
phynity_apply_sanitizers(phynity_demo)
