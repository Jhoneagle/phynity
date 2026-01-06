if(NOT BUILD_TESTING)
	return()
endif()

find_package(Catch2 REQUIRED CONFIG)

add_library(phynity_tests INTERFACE)
add_library(phynity::tests ALIAS phynity_tests)

target_link_libraries(phynity_tests INTERFACE phynity::core)
target_include_directories(phynity_tests INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/..
	${CMAKE_CURRENT_LIST_DIR}/../src
)

# New structured test suites
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/UnitTests/CMakeLists.txt)
	add_subdirectory(UnitTests)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/ValidationTests/CMakeLists.txt)
	add_subdirectory(ValidationTests)
endif()

# Optional future benchmarks
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/Benchmarks/CMakeLists.txt)
	add_subdirectory(Benchmarks)
endif()
