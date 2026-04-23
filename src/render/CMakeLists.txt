if(PHYNITY_BUILD_RENDER)
	find_package(imgui CONFIG REQUIRED)
	find_package(glfw3 CONFIG REQUIRED)
	find_package(OpenGL REQUIRED)

	add_library(phynity_render STATIC
		window.cpp
		imgui_context.cpp
	)
	add_library(phynity::render ALIAS phynity_render)
	set_target_properties(phynity_render PROPERTIES EXPORT_NAME render)

	target_include_directories(phynity_render PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/render>
	)

	target_link_libraries(phynity_render
		PUBLIC phynity::core
		PRIVATE imgui::imgui glfw OpenGL::GL
	)

	target_compile_definitions(phynity_render PUBLIC PHYNITY_HAS_RENDER=1)

	phynity_target_defaults(phynity_render)
	phynity_apply_warnings(phynity_render)
	phynity_apply_sanitizers(phynity_render)
else()
	# Fallback: empty interface library for headless builds
	add_library(phynity_render INTERFACE)
	add_library(phynity::render ALIAS phynity_render)
	set_target_properties(phynity_render PROPERTIES EXPORT_NAME render)

	target_include_directories(phynity_render INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/render>
	)

	target_link_libraries(phynity_render INTERFACE phynity::core)

	phynity_target_defaults(phynity_render)
	phynity_apply_warnings(phynity_render)
	phynity_apply_sanitizers(phynity_render)
endif()
