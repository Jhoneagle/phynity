#!/usr/bin/env bash
set -euo pipefail

# Usage: ./tools/format.sh [--check]
# Format all C++ source files in src/ and tests/ directories
# --check: Only check formatting without modifying files (exit code 1 if changes needed)

cd "$(dirname "$0")/.."

if [[ "${1:-}" == "--check" ]]; then
    echo "Checking code formatting..."
    if ! find src tests -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) -print0 | xargs -0 clang-format --dry-run -Werror; then
        echo ""
        echo "Code formatting issues found. Run './tools/format.sh' to fix them."
        exit 1
    fi
    echo "All files are properly formatted."
else
    echo "Formatting all C++ files..."
    find src tests -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) -print0 | xargs -0 clang-format -i
    echo "Done."
fi
