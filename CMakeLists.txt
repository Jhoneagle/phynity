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
set(PHYNITY_SANITIZER_MODE "address-undefined" CACHE STRING "Sanitizer mode: address-undefined or thread")
set_property(CACHE PHYNITY_SANITIZER_MODE PROPERTY STRINGS "address-undefined" "thread")
option(PHYNITY_ENABLE_LTO "Enable link-time optimization where supported" OFF)
option(PHYNITY_BUILD_RENDER "Build render module with ImGui/GLFW/OpenGL" ON)
option(PHYNITY_BUILD_TESTS "Build tests" ON)

if(NOT PHYNITY_SANITIZER_MODE STREQUAL "address-undefined" AND NOT PHYNITY_SANITIZER_MODE STREQUAL "thread")
    message(FATAL_ERROR "Invalid PHYNITY_SANITIZER_MODE='${PHYNITY_SANITIZER_MODE}'. Allowed: address-undefined, thread")
endif()

# Platform-specific overrides
if(MINGW AND PHYNITY_ENABLE_LTO)
    message(STATUS "[LTO] Disabling on MinGW: lto1 tooling not available")
    set(PHYNITY_ENABLE_LTO OFF CACHE BOOL "LTO disabled on MinGW" FORCE)
endif()

# Helper modules
include(cmake/compiler_options.cmake OPTIONAL)
include(cmake/warnings.cmake OPTIONAL)
include(cmake/sanitizers.cmake OPTIONAL)
include(GNUInstallDirs)

# Engine library components
add_subdirectory(src/core)
add_subdirectory(src/platform)
add_subdirectory(src/render)

# Umbrella library target: consumers link phynity::phynity to get everything
add_library(phynity_engine INTERFACE)
add_library(phynity::phynity ALIAS phynity_engine)
set_target_properties(phynity_engine PROPERTIES EXPORT_NAME phynity)
target_link_libraries(phynity_engine INTERFACE
    phynity::core
    phynity::platform
    phynity::render
)

# Install rules
include(CMakePackageConfigHelpers)

install(TARGETS phynity_engine phynity_core phynity_platform phynity_render
    EXPORT phynityTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY src/core/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/core
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
    PATTERN "CMakeLists.txt" EXCLUDE
)
install(DIRECTORY src/platform/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/platform
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
    PATTERN "CMakeLists.txt" EXCLUDE
)
install(DIRECTORY src/render/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/render
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
    PATTERN "CMakeLists.txt" EXCLUDE
)

install(EXPORT phynityTargets
    FILE phynityTargets.cmake
    NAMESPACE phynity::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/phynity
)

configure_package_config_file(
    cmake/phynityConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/phynityConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/phynity
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/phynityConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/phynityConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/phynityConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/phynity
)

# Sandbox demo application
option(PHYNITY_BUILD_SANDBOX "Build sandbox demos" ON)
if(PHYNITY_BUILD_SANDBOX)
    add_subdirectory(sandbox)
endif()

# Tests
if(PHYNITY_BUILD_TESTS)
    include(CTest)
    add_subdirectory(tests)
endif()