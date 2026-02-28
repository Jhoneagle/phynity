#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./tools/test.sh [preset] [filter]
# Examples:
#   ./tools/test.sh            # debug preset, all tests
#   ./tools/test.sh release    # release preset, all tests
#   ./tools/test.sh debug core # run tests matching 'core'
#   ./tools/test.sh debug golden-compare # run golden tests (compare)
#   ./tools/test.sh debug golden         # run golden tests (capture)

preset="${1:-debug}"
filter="${2:-}"
filter_regex="$filter"
cmake_extra_flags=""
case "$filter" in
  unit|Unit|UNIT) filter_regex='^unit\.' ;;
  validation|Validation|VALIDATION) filter_regex='^validation\.' ;;
  golden-compare|Golden-compare|GOLDEN-COMPARE)
    filter_regex='golden'
    ;;
  golden|Golden|GOLDEN)
    filter_regex='[golden]'
    cmake_extra_flags="-DGOLDEN_CAPTURE_MODE=ON"
    ;;
esac

# Build first to ensure tests are compiled
script_dir="$(cd "$(dirname "$0")" && pwd)"
CMAKE_EXTRA_FLAGS="$cmake_extra_flags" bash "$script_dir/build.sh" "$preset"

# Run tests via CTest preset
if [[ -n "$filter_regex" ]]; then
  ctest --preset "$preset" -R "$filter_regex" --output-on-failure || true
else
  ctest --preset "$preset" --output-on-failure || true
fi

# Fallback for Windows/MSYS environments where CTest may fail to launch .exe
os="$(uname -s || echo unknown)"
if [[ "$os" == MINGW* || "$os" == MSYS* || "$os" == CYGWIN* ]]; then
  tests_root="build/$preset/tests"
  if [[ -d "$tests_root" ]]; then
    shopt -s nullglob
    mapfile -t exes < <(find "$tests_root" -type f -name "*_test.exe" 2>/dev/null || true)
    for exe in "${exes[@]}"; do
      bn="$(basename "$exe")"
      run_this=true
      if [[ -n "$filter" ]]; then
        case "$filter" in
          unit|Unit|UNIT)
            [[ "$exe" =~ \\UnitTests\\|/UnitTests/ ]] || run_this=false ;;
          validation|Validation|VALIDATION)
            [[ "$exe" =~ \\ValidationTests\\|/ValidationTests/ ]] || run_this=false ;;
          *)
            [[ "$bn" == *"$filter"* ]] || run_this=false ;;
        esac
      fi
      if $run_this; then
        if command -v cmd.exe >/dev/null 2>&1; then
          if command -v cygpath >/dev/null 2>&1; then
            win_path="$(cygpath -w "$exe")"
          else
            win_path="$exe"
          fi
          cmd.exe /c "$win_path"
        else
          "$exe"
        fi
      fi
    done
  fi
fi
