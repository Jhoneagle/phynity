# Physics Engine / Simulation Project

A modern C++ physics engine and simulation playground focused on
learning, experimentation, and long-term evolution toward an
industry-grade solution.

## Goals

- Use modern C++ (C++20+) and modern CMake
- Explore physics simulation and numerical methods
- Learn concurrency and parallel programming
- Maintain clean architecture and production-quality practices
- Support interactive experimentation and visualization

## Non-goals (for now)

- Game-specific features
- Asset pipelines
- Full real-time rendering engine

## Build

This project uses CMake and vcpkg (manifest mode).

```bash
cmake --preset debug
cmake --build --preset debug
```

## Development Tools

### Code Formatting

This project uses clang-format 18 for code style consistency across all platforms.

**First-time setup (enable pre-commit hook):**

```bash
git config core.hooksPath .githooks
```

The pre-commit hook automatically formats staged C++ files before each commit, ensuring consistent formatting. To bypass temporarily: `git commit --no-verify`

**Manual formatting:**

```bash
# Format all code
./tools/format.sh          # Linux/macOS
.\tools\format.bat           # Windows

# Check formatting without modifying
./tools/format.sh --check
.\tools\format.bat --check
```

**Installing clang-format 18:**

```bash
# Linux (Ubuntu/Debian)
wget https://apt.llvm.org/llvm.sh
chmod +x llvm.sh
sudo ./llvm.sh 18
sudo apt-get install clang-format-18

# Windows (via LLVM installer)
# Download from: https://github.com/llvm/llvm-project/releases/tag/llvmorg-18.1.8
# Or use winget: winget install LLVM.LLVM

# macOS
brew install llvm@18
```

### Static Analysis

This project uses clang-tidy for local static analysis and in CI.

```bash
# Run the same style of static analysis used by CI
./tools/static_analysis.sh       # Linux/macOS
.\tools\static_analysis.bat      # Windows

# Choose a different preset if needed
./tools/static_analysis.sh debug
.\tools\static_analysis.bat debug
```

The script configures the selected CMake preset, ensures `compile_commands.json` exists, and runs `clang-tidy` over `src` and `tests` (`*.cpp`, `*.hpp`, `*.h`) using the repository's `.clang-tidy` settings.
