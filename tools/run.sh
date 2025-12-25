#!/usr/bin/env bash
set -euo pipefail

# Usage: ./tools/run.sh [preset]
# Presets: debug (default), release

preset="${1:-debug}"
os="$(uname -s || echo unknown)"

exe_path="build/$preset/src/app/phynity_demo"

if [[ "$os" == MINGW* || "$os" == MSYS* || "$os" == CYGWIN* ]]; then
  exe_path+=".exe"
  # Prefer running via cmd.exe to avoid MSYS path quirks
  if command -v cmd.exe >/dev/null 2>&1; then
    # Convert to Windows path if available
    if command -v cygpath >/dev/null 2>&1; then
      win_path="$(cygpath -w "$exe_path")"
    else
      # Fallback: build Windows-style path using pwd -W if available
      if command -v pwd >/dev/null 2>&1; then
        base_win="$(pwd -W 2>/dev/null || echo "")"
      else
        base_win=""
      fi
      if [[ -n "$base_win" ]]; then
        # Replace forward slashes with backslashes
        rel_win="${exe_path//\//\\}"
        win_path="${base_win}\\${rel_win}"
      else
        win_path="$exe_path" # Last resort
      fi
    fi
    exec cmd.exe /c "$win_path"
  fi
fi

# Non-Windows or direct execution
exec "$exe_path"
