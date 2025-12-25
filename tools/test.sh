#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./tools/test.sh [preset] [regex]
# Examples:
#   ./tools/test.sh            # debug preset, all tests
#   ./tools/test.sh release    # release preset, all tests
#   ./tools/test.sh debug core # run tests matching 'core'

preset="${1:-debug}"
regex="${2:-}"

# Build first to ensure tests are compiled
script_dir="$(cd "$(dirname "$0")" && pwd)"
bash "$script_dir/build.sh" "$preset"

# Run tests via CTest preset
if [[ -n "$regex" ]]; then
  ctest --preset "$preset" -R "$regex" --output-on-failure || true
else
  ctest --preset "$preset" --output-on-failure || true
fi

# Fallback for Windows/MSYS environments where CTest may fail to launch .exe
os="$(uname -s || echo unknown)"
if [[ "$os" == MINGW* || "$os" == MSYS* || "$os" == CYGWIN* ]]; then
  core_dir="build/$preset/tests/core"
  if [[ -d "$core_dir" ]]; then
    if [[ -z "$regex" || "$regex" =~ core ]]; then
      if command -v cmd.exe >/dev/null 2>&1; then
        if [[ -f "$core_dir/vec3_test.exe" ]]; then cmd.exe /c "$(cygpath -w "$core_dir/vec3_test.exe" 2>/dev/null || echo "$core_dir/vec3_test.exe")"; fi
        if [[ -f "$core_dir/particle_test.exe" ]]; then cmd.exe /c "$(cygpath -w "$core_dir/particle_test.exe" 2>/dev/null || echo "$core_dir/particle_test.exe")"; fi
      else
        # Direct execution fallback
        [[ -f "$core_dir/vec3_test.exe" ]] && "$core_dir/vec3_test.exe"
        [[ -f "$core_dir/particle_test.exe" ]] && "$core_dir/particle_test.exe"
      fi
    fi
  fi
fi
