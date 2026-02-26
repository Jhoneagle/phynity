# Development Roadmap

This roadmap is organized by time horizon and anchored to the current architecture: `core` owns math/physics and determinism, `platform` owns timing and threading, `render` is optional visualization, and `app` is orchestration. Items marked (S) are short-term deliverables, (M) mid-term, and (L) long-term.

## Near Term (0–3 months)
- Core math bedrock (S): vectors/matrices/quaternions, linear algebra routines, trig helpers, numerical stability checks, small-angle approximations.✅
- Calculus utilities (S): forward/central differences, simple integrators (Euler, semi-implicit), curve fitting for derivatives; integration harness for regression tests.✅
- Particle simulation v2 (S): variable timestep support with determinism guardrails, basic gravity/drag fields, per-particle material presets.✅
- Parallel core bootstrap (S): minimal job system, work-stealing queues, thread-safe arenas; deterministic scheduling mode for tests.
- Collision scaffolding (S): broadphase spatial grid, narrowphase for spheres/AABBs, contact manifold data structures.✅
- Observability (S): profiling hooks in hot loops, frame markers, sanity-check assertions for energy/momentum drift.✅
- Testing (S): golden tests for math primitives, particle integration, and deterministic stepping; CI smoke targets.✅

## Mid Term (3–9 months)
- Rigid body MVP (M): inertia tensors, angular integration, basic constraints (fixed, hinge), Baumgarte stabilization; simple stacking demo.
- Collision and constraints (M): SAT for convex hulls, GJK/EPA for general shapes, contact caching, impulse-based and PGS solvers.
- Continuous physics (M): CCD sweeps for fast movers, time-of-impact resolution, restitution and friction modeling.
- Fields and forces (M): configurable gravity fields, wind/drag volumes, springs/dampers, buoyancy for simple fluids.
- Thermodynamics hooks (M): temperature as scalar field, heat diffusion prototype, energy accounting to support conservation checks.
- Electromagnetism starter (M): charged particles with Coulomb forces, simple magnetic field lines, Lorentz force integration.
- Fluids track (M): particle-based fluids (SPH/PBF) prototype, viscosity/tension terms, pressure projection experiments.
- CI/CD infrastructure (M): GitHub Actions workflows for Windows/Linux/macOS, automated test gating on PRs, flakiness detection, artifact archival; deterministic test retry logic for intermittent failures.
- Performance monitoring (M): automated benchmark regression detection with per-test thresholds, historical tracking (CSV/DB), performance dashboard for frame time/memory trends, alerts for > 5% regressions.
- Platform and concurrency (M): task graph builder, fiber-backed jobs, pinned tasks for cache locality; deterministic replay of job schedules.
- Tooling and UI (M): in-engine debug HUD, timeline scrubber for stepping, detachable inspectors for bodies/constraints.
- Data and serialization (M): snapshot save/load, replay files, binary + JSON interchange for tests.

## Long Term (9–18 months)
- Advanced rigid bodies (L): articulated bodies, joint limits, motors, breakable constraints, soft constraints blending.
- Deformables and soft bodies (L): mass-spring systems, position-based dynamics constraints, cloth/rope with self-collision handling.
- Fluid and gas expansions (L): incompressible solver with pressure projection, vorticity confinement, multiphase interactions, basic combustion hooks.
- Electromagnetism and fields (L): grid-based field solvers, induction effects, coupling with conductive materials.
- Thermodynamics and energy (L): advection-diffusion solvers, phase-change placeholders, rigorous conservation diagnostics.
- Relativity/theoretical extensions (L): optional relativistic kinematics layer for high-velocity scenarios; experiment-only path, isolated from core.
- Particle physics sandbox (L): stochastic decay rules, simple probability-driven event system; isolated module for experimentation.
- Chemistry hooks (L): material library (elements/molecules), simple reaction rules for energy release/absorption; opt-in module.
- Large-scale optimization (L): SoA refactors, cache-friendly contact persistence, SIMD kernels, GPU/compute-backend investigation.
- Parallelism at scale (L): NUMA-aware scheduling, task batching for thousands of bodies, deterministic lock-free data paths where feasible.

## Quality, Process, and Docs (ongoing)
- Determinism contract: document fixed-point vs floating-point modes, reproducibility expectations, tolerance budgets.
- Validation: benchmark scenes for collisions, constraints, fluids, and EM fields with expected energy/momentum ranges.
- Architecture alignment: keep physics/math in `core`; `platform` owns timing/threads; `render` remains optional; `app` orchestrates scenarios.
- Documentation: expand architecture and module READMEs as new systems land; keep troubleshooting and profiling guides up to date.
- Testing cadence: unit tests for math/forces, property-based tests for invariants, replay-based regression tests for determinism and performance.
