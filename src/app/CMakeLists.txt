add_library(phynity_app INTERFACE)
add_library(phynity::app ALIAS phynity_app)

target_include_directories(phynity_app INTERFACE
	${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(phynity_app INTERFACE
	phynity::core
	phynity::platform
	phynity::render
)

phynity_target_defaults(phynity_app)
phynity_apply_warnings(phynity_app)
phynity_apply_sanitizers(phynity_app)

# Executable
add_executable(phynity_demo main.cpp)
target_link_libraries(phynity_demo PRIVATE phynity::app)
phynity_target_defaults(phynity_demo)
phynity_apply_warnings(phynity_demo)
phynity_apply_sanitizers(phynity_demo)
