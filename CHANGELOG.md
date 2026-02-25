# Changelog

## 0.1.0 (2026-02-25)

### Added
- Typed collision shapes in DynamisCollision 1.1.1 integration path.
- ODE4J backend feature completion:
  - shape adapter for all `ShapeType` variants
  - constraint system + builders
  - vehicle system + events
  - character controller + landing hints/events
  - ragdoll blend system + get-up hints
  - snapshot/restore + determinism gate
  - debug renderer via DynamisGPU
- Jolt backend feature completion:
  - core world/body/raycast/events parity
  - character parity (`CharacterVirtual` path)
  - vehicle parity (`VehicleConstraint` path)
  - ragdoll parity + blend/hints
  - snapshot/restore parity
  - deterministic mode (1 thread) and multithread mode (configurable)
  - optional stress gate `JoltParallelStabilityTest`
- Step 13 benchmark module (`dynamisphysics-bench`) with JMH suites:
  - `RigidBodyStepBenchmark`
  - `RaycastBenchmark`
  - `VehicleBenchmark`

### Changed
- Documented gate commands for ODE4J, Jolt parity, and Jolt stress runs.
- Added helper scripts:
  - `scripts/gate-ode4j.sh`
  - `scripts/gate-jolt.sh`
  - `scripts/gate-jolt-stress.sh`
  - `scripts/bench.sh`

### Notes
- Jolt JNI execution remains JDK 25-only in this repository.
- Snapshot parity is behavioural across backends; byte identity is backend-local.
- Deterministic mode defaults to single-thread Jolt stepping; override with `-Djolt.threads=<N>`.
