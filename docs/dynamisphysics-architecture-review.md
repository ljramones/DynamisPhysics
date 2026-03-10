# DynamisPhysics Architecture Review

Date: 2026-03-10  
Scope: Deep boundary ratification for `DynamisPhysics` (review/documentation only)

## 1. Repo Overview

Observed modules:

- `dynamisphysics-api`
- `dynamisphysics-ode4j`
- `dynamisphysics-jolt`
- `dynamisphysics-test`
- `dynamisphysics-bench`

Observed implementation shape:

- `dynamisphysics-api` defines simulation contracts (`PhysicsWorld`, world/body/constraint/query/event/config descriptors, factory registration hooks).
- `dynamisphysics-ode4j` and `dynamisphysics-jolt` implement backend-specific simulation worlds and adapters.
- `dynamisphysics-test` provides mocks/harness/assertion/replay utilities.
- `dynamisphysics-bench` provides JMH benchmarks and regression tooling.

Dependency signals from poms/code:

- API depends on `vectrix`, `animis`, and `org.dynamiscollision:collision_detection` contracts.
- Backends depend on collision + meshforge (shape extraction support), and `dynamisphysics-ode4j` also depends on `dynamis-gpu-api` for debug rendering path.
- No direct dependencies on `DynamisWorldEngine`, `DynamisECS`, `DynamisSceneGraph`, `DynamisSession`, `DynamisLightEngine`, or `DynamisScripting`.

## 2. Strict Ownership Statement

### What DynamisPhysics should own

- Physics simulation authority for bodies/constraints/contacts/character/vehicle/ragdoll dynamics.
- Simulation step/solver contracts and runtime world stepping at physics scope.
- Physics-side runtime queries tied to simulation state (raycast/shapecast/overlap against physics world).
- Physics events and contact lifecycle publication from simulation outcomes.
- Determinism/snapshot/replay mechanics for physics-state verification.

### What is appropriate for a physics subsystem

- Backend-agnostic simulation API plus backend implementations.
- Determinism/parity instrumentation and benchmark validation.
- Physics-specific debug data production (contacts, wireframe primitives, stats) as optional diagnostics.

### What DynamisPhysics must never own

- Global world authority/orchestration (belongs to `DynamisWorldEngine`).
- ECS substrate ownership and entity-state authority (belongs to `DynamisECS` + world orchestration).
- Scene hierarchy ownership (belongs to `DynamisSceneGraph`).
- Gameplay/system orchestration policy and scripting policy.
- Render planning/GPU execution ownership as a primary concern.
- Session persistence authority.

## 3. Dependency Rules

### Allowed dependencies for DynamisPhysics

- Math/animation/collision substrate dependencies needed for simulation contracts.
- Backend-specific native/engine dependencies needed to implement physics simulation.
- Test/bench support dependencies for determinism/parity/replay validation.

### Forbidden dependencies for DynamisPhysics

- Direct world orchestration authority dependencies (`DynamisWorldEngine` policy layer).
- ECS/scene/session/scripting ownership dependencies.
- Render-planning policy dependencies (`DynamisLightEngine` planner concerns).

### Who may depend on DynamisPhysics

- `DynamisWorldEngine` as orchestration layer consuming physics simulation.
- `DynamisECS` systems that project ECS state into/out of physics world via world-layer integration.
- Feature subsystems that consume physics outputs/events under world orchestration.

### Boundary requirements

- Physics is simulation authority; WorldEngine remains orchestration authority.
- Collision remains geometry/query substrate; Physics owns dynamic simulation and contact resolution lifecycle.
- ECS is state substrate, not physics solver owner.

## 4. Public vs Internal Boundary

### Canonical public surface (recommended)

- `dynamisphysics-api` packages (`api.world`, `api.body`, `api.constraint`, `api.event`, `api.query`, `api.config`, etc.).
- `PhysicsWorldFactory` registration boundary for backend provisioning.

### Internal/implementation surface (should remain internal)

- Backend-specific registries/adapters/controllers in `ode4j`/`jolt` modules.
- Backend debug rendering internals.
- Replay harness internals in test module.

### Boundary concerns

- Backend modules export additional implementation packages (`*.world`, `*.debug`), which can freeze backend internals as external API.
- `dynamisphysics-test` exports many utility packages; useful for tests but can become unintended dependency surface for production modules.

## 5. Policy Leakage / Overlap Findings

## Major clean boundaries confirmed

- Strong simulation-centric API and backend split.
- No direct ownership drift into WorldEngine/ECS/SceneGraph/session/scripting layers in code dependencies.
- Clear determinism/parity/replay discipline consistent with physics simulation authority.

## Policy leakage / overlap identified

- **DynamisCollision overlap risk (moderate):** API/backends depend directly on collision substrate types (`CollisionShape`) and mesh extraction helpers. This is expected but boundary should stay: collision defines shape/query substrate, physics owns dynamic simulation + contact/solver lifecycle.
- **DynamisWorldEngine overlap risk (watch):** `PhysicsWorld.step(...)`, pause/resume, and time-scale exist in physics scope; keep global tick orchestration and subsystem sequencing in WorldEngine.
- **Render/debug overlap risk (moderate):** `dynamisphysics-ode4j` debug path (`Ode4jDebugRenderer`) depends on `dynamis-gpu-api` and records GPU staging/commands. This is a boundary hotspot against LightEngine/GPU ownership and should remain optional/debug-only unless moved behind clearer debug-consumption interfaces later.
- **Potential gameplay feature sprawl risk (watch):** broad feature surface (vehicles/characters/ragdolls/economy-facing tests) is still physics-adjacent but should avoid absorbing gameplay-rule policy.

## 6. Ratification Result

**Judgment: ratified with constraints**

Why:

- The repo clearly functions as physics simulation authority with coherent backend and API structure.
- Major architectural boundaries are mostly intact versus World/ECS/Scene/Scripting ownership.
- Constraints are needed around collision boundary clarity and debug render/GPU coupling in backend debug modules.

## 7. Recommended Next Step

1. Keep physics as simulation authority only; preserve WorldEngine as global orchestrator.
2. Explicitly ratify Physics ↔ Collision boundary next:
   - Collision: shape/query substrate.
   - Physics: dynamic simulation, contacts, solver, simulation queries.
3. Review and constrain backend debug-render hooks so they remain diagnostics and do not evolve into render-planning ownership.
4. Next repo to review: **DynamisCollision**.

---

This document is a boundary-ratification review artifact. It does not perform refactors in this pass.
