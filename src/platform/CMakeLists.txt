add_library(phynity_platform INTERFACE)
add_library(phynity::platform ALIAS phynity_platform)
set_target_properties(phynity_platform PROPERTIES EXPORT_NAME platform)

target_include_directories(phynity_platform INTERFACE
	$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

phynity_target_defaults(phynity_platform)
phynity_apply_warnings(phynity_platform)
phynity_apply_sanitizers(phynity_platform)
