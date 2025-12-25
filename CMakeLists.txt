cmake_minimum_required(VERSION 3.27)

project(
    phynity
    VERSION 0.1.0
    DESCRIPTION "Physics engine / simulation playground"
    LANGUAGES CXX
)

# Global defaults
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Options
option(PHYNITY_ENABLE_WARNINGS "Enable project warnings" ON)
option(PHYNITY_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
option(PHYNITY_ENABLE_SANITIZERS "Enable sanitizers when supported" OFF)
option(PHYNITY_ENABLE_LTO "Enable link-time optimization where supported" OFF)
option(PHYNITY_BUILD_TESTS "Build tests" ON)

# Platform-specific overrides
if(MINGW AND PHYNITY_ENABLE_LTO)
    message(STATUS "[LTO] Disabling on MinGW: lto1 tooling not available")
    set(PHYNITY_ENABLE_LTO OFF CACHE BOOL "LTO disabled on MinGW" FORCE)
endif()

# Helper modules
include(cmake/compiler_options.cmake OPTIONAL)
include(cmake/warnings.cmake OPTIONAL)
include(cmake/sanitizers.cmake OPTIONAL)

# Project components
add_subdirectory(src/core)
add_subdirectory(src/platform)
add_subdirectory(src/render)
add_subdirectory(src/app)

# Tests
if(PHYNITY_BUILD_TESTS)
    include(CTest)
    add_subdirectory(tests)
endif()