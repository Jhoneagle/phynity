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
