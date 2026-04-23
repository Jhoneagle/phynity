#!/usr/bin/env bash
set -euo pipefail

# Usage: ./tools/static_analysis.sh [preset]
# Run clang-tidy using the compile_commands.json from build/<preset>
# Presets: release (default), debug, debug-asan, debug-tsan, release-lto

cd "$(dirname "$0")/.."

preset="${1:-release}"
build_dir="build/$preset"
log_dir="$build_dir/logs"
mkdir -p "$log_dir"
log_file="$log_dir/static_analysis.log"

# Write all output to a timestamped log file so long runs are preserved.
exec > "$log_file" 2>&1

echo "Writing full static analysis output to: $log_file"

if ! command -v clang-tidy >/dev/null 2>&1; then
    echo "clang-tidy not found in PATH."
    exit 1
fi

if ! command -v cmake >/dev/null 2>&1; then
    echo "cmake not found in PATH."
    exit 1
fi

echo "Configuring preset '$preset' for static analysis..."
cmake --preset "$preset"

if [[ ! -f "$build_dir/compile_commands.json" ]]; then
    echo "Generating compile_commands.json for '$preset'..."
    cmake --build --preset "$preset"
fi

echo "Running clang-tidy over src/ and tests/ (*.{cpp,hpp,h}) using $build_dir/compile_commands.json..."
failed=0
while read -r file; do
    echo "Analyzing: $file"
    if ! clang-tidy -p "$build_dir" "$file"; then
        failed=1
    fi
done < <(find src sandbox tests -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) | sort)

if [[ $failed -ne 0 ]]; then
    echo ""
    echo "Static analysis issues found."
    exit 1
fi

echo "Static analysis complete."