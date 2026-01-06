add_library(phynity_render INTERFACE)
add_library(phynity::render ALIAS phynity_render)

target_include_directories(phynity_render INTERFACE
	${CMAKE_CURRENT_SOURCE_DIR}/src/render
)

target_link_libraries(phynity_render INTERFACE phynity::core)

phynity_target_defaults(phynity_render)
phynity_apply_warnings(phynity_render)
phynity_apply_sanitizers(phynity_render)
