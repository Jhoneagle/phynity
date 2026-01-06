add_library(phynity_core INTERFACE)
add_library(phynity::core ALIAS phynity_core)

target_include_directories(phynity_core INTERFACE
	${CMAKE_SOURCE_DIR}/src/core
)

phynity_target_defaults(phynity_core)
phynity_apply_warnings(phynity_core)
phynity_apply_sanitizers(phynity_core)
