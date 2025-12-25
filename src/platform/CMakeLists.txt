add_library(phynity_platform INTERFACE)
add_library(phynity::platform ALIAS phynity_platform)

target_include_directories(phynity_platform INTERFACE
	${CMAKE_CURRENT_SOURCE_DIR}
)

phynity_target_defaults(phynity_platform)
phynity_apply_warnings(phynity_platform)
phynity_apply_sanitizers(phynity_platform)
