# Architecture Overview

## High-Level Structure

The project is organized into clearly separated layers:

- Core simulation (math, physics, memory, jobs)
- Platform abstractions
- Optional rendering and visualization
- Application-level executables

The simulation core is fully decoupled from rendering and platform code.

## Core Principles

- Data-oriented design where appropriate
- Clear ownership and lifetime rules
- Deterministic simulation steps when possible
- Explicit dependencies between modules

## Module Responsibilities

### core/
Contains all platform-agnostic logic:
- Math and numerical utilities
- Physics simulation
- Memory management
- Job system and concurrency primitives

### platform/
Thin abstractions over OS and compiler features such as:
- Threading
- Timing
- CPU feature detection

### render/
Visualization and debugging tools only.
Depends on core, never the other way around.

### app/
Executable entry points and experiment sandboxes.
Minimal logic; orchestration only.

## Dependency Rules

- core must not depend on platform or render
- render may depend on core
- app may depend on everything
