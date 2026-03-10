#!/usr/bin/env bash
set -euo pipefail

# Usage: ./tools/build.sh [preset]
# Presets: debug (default), release
# Env overrides:
#   PHYNITY_SANITIZERS=ON|OFF|auto (default: auto)
#   PHYNITY_WARNINGS_AS_ERRORS=ON|OFF (default: ON)
#   VCPKG_TARGET_TRIPLET=<triplet> (default auto by platform)
#   CMAKE_EXTRA_FLAGS="-DVAR=value" for additional CMake flags
#   CLEAN=true to remove build/<preset> before configuring
#   RECONFIGURE=true forces reconfigure (cmake --preset still configures if needed)

# Limit vcpkg parallelism to avoid file system contention
export VCPKG_MAX_CONCURRENCY=4

preset="${1:-debug}"
sanitizers="${PHYNITY_SANITIZERS:-auto}"
werror="${PHYNITY_WARNINGS_AS_ERRORS:-ON}"
os="$(uname -s || echo unknown)"
arch="$(uname -m || echo unknown)"
triplet="${VCPKG_TARGET_TRIPLET:-}"

# Decide sanitizer default based on platform
if [[ "$sanitizers" == "auto" ]]; then
  if [[ "$os" == MINGW* || "$os" == MSYS* || "$os" == CYGWIN* ]]; then
    sanitizers="OFF"  # MinGW on Windows typically lacks usable ASan/UBSan
  else
    sanitizers="ON"
  fi
fi

if [[ -z "$triplet" ]]; then
  case "$os" in
    Linux*)
      triplet="x64-linux"
      ;;
    Darwin*)
      if [[ "$arch" == "arm64" ]]; then
        triplet="arm64-osx"
      else
        triplet="x64-osx"
      fi
      ;;
    MINGW*|MSYS*|CYGWIN*)
      triplet="x64-mingw-static"
      ;;
  esac
fi

# Optional clean
if [[ "${CLEAN:-false}" == "true" ]]; then
  rm -rf "build/$preset"
fi

# Compose extra cache variables
extra_cache=("-DPHYNITY_ENABLE_SANITIZERS=$sanitizers")
extra_cache+=("-DPHYNITY_WARNINGS_AS_ERRORS=$werror")
if [[ -n "$triplet" ]]; then
  extra_cache+=("-DVCPKG_TARGET_TRIPLET=$triplet")
fi

# Add any additional CMake flags passed via environment
if [[ -n "${CMAKE_EXTRA_FLAGS:-}" ]]; then
  extra_cache+=($CMAKE_EXTRA_FLAGS)
fi

# Hook vcpkg toolchain if available
if [[ -n "${VCPKG_ROOT:-}" && -f "vcpkg.json" ]]; then
  extra_cache+=("-DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake")
fi

# Configure + build
cmake --preset "$preset" "${extra_cache[@]}"
cmake --build --preset "$preset"

echo "Build finished: build/$preset"
