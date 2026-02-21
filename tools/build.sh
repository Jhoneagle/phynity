#!/usr/bin/env bash
set -euo pipefail

# Usage: ./tools/build.sh [preset]
# Presets: debug (default), release
# Env overrides:
#   PHYNITY_SANITIZERS=ON|OFF|auto (default: auto)
#   CLEAN=true to remove build/<preset> before configuring
#   RECONFIGURE=true forces reconfigure (cmake --preset still configures if needed)

# Limit vcpkg parallelism to avoid file system contention
export VCPKG_MAX_CONCURRENCY=4

preset="${1:-debug}"
sanitizers="${PHYNITY_SANITIZERS:-auto}"
os="$(uname -s || echo unknown)"

# Decide sanitizer default based on platform
if [[ "$sanitizers" == "auto" ]]; then
  if [[ "$os" == MINGW* || "$os" == MSYS* || "$os" == CYGWIN* ]]; then
    sanitizers="OFF"  # MinGW on Windows typically lacks usable ASan/UBSan
  else
    sanitizers="ON"
  fi
fi

# Optional clean
if [[ "${CLEAN:-false}" == "true" ]]; then
  rm -rf "build/$preset"
fi

# Compose extra cache variables
extra_cache=("-DPHYNITY_ENABLE_SANITIZERS=$sanitizers")

# Hook vcpkg toolchain if available
if [[ -n "${VCPKG_ROOT:-}" && -f "vcpkg.json" ]]; then
  extra_cache+=("-DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake")
fi

# Configure + build
cmake --preset "$preset" "${extra_cache[@]}"
cmake --build --preset "$preset"

echo "Build finished: build/$preset"
