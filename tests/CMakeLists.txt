if(NOT BUILD_TESTING)
	return()
endif()

add_library(phynity_tests INTERFACE)
add_library(phynity::tests ALIAS phynity_tests)

target_link_libraries(phynity_tests INTERFACE phynity::core)

# Add per-module test suites when they appear.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/core/CMakeLists.txt)
	add_subdirectory(core)
endif()
