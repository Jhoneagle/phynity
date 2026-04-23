# Phynity - Physics Engine

A modern C++ physics engine library and simulation sandbox focused on
learning, experimentation, and long-term evolution toward an
industry-grade solution.

## Project Structure

```
phynity/
├── src/              Engine library (the physics engine itself)
│   ├── core/         Math, physics, jobs, serialization, diagnostics
│   ├── platform/     Threading and OS abstractions
│   └── render/       Optional visualization (ImGui/GLFW/OpenGL)
├── sandbox/          Interactive sandbox application with debug UI
├── tests/            Unit and validation tests
├── cmake/            Build helpers and install config
└── tools/            Dev scripts (format, build, run, etc.)
```

The engine (`src/`) is a standalone library. The sandbox (`sandbox/`) is a
separate executable that consumes the library for manual testing and demos.

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

## Using as a Library

After building, install the library:

```bash
cmake --install build/release --prefix /path/to/install
```

Then in your own CMake project:

```cmake
find_package(phynity 0.1 REQUIRED)
target_link_libraries(my_app PRIVATE phynity::phynity)
```

```cpp
#include <core/math/vectors/vec3.hpp>
#include <core/physics/particles/particle_system.hpp>
#include <platform/threading.hpp>
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

The script configures the selected CMake preset, ensures `compile_commands.json` exists, and runs `clang-tidy` over `src`, `sandbox` and `tests` (`*.cpp`, `*.hpp`, `*.h`) using the repository's `.clang-tidy` settings.

## Interactive Sandbox

The sandbox application provides an interactive windowed environment for
visualizing and debugging physics simulations.

### Dependencies

- **Dear ImGui** — immediate-mode debug UI
- **GLFW** — cross-platform window management and input
- **OpenGL 3.3** — wireframe rendering

All installed via vcpkg. To build without the render module (e.g., for CI or
headless environments), set `-DPHYNITY_BUILD_RENDER=OFF`.

### Running

```bash
# Interactive windowed mode (default)
./build/release/phynity_sandbox

# Headless console mode (runs all scenarios and prints diagnostics)
./build/release/phynity_sandbox --headless
```

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| F1 | Toggle help overlay |
| F2 | Toggle ImGui demo window |
| F3 | Toggle debug HUD |
| Space | Play / Pause simulation |
| Left Arrow | Step backward (when paused) |
| Right Arrow | Step forward (when paused) |
| `[` / `]` | Decrease / Increase speed |
| Left Drag | Orbit camera |
| Right Drag | Pan camera |
| Scroll | Zoom camera |
| Home | Reset camera |
| Left Click | Select body (opens inspector) |
