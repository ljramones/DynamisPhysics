# DynamisPhysics

DynamisPhysics is a dual-backend physics platform for the Dynamis ecosystem.
It exposes one API and supports two implementations:

- **ODE4J backend**: deterministic-first, pure Java
- **Jolt backend**: native JNI backend with deterministic mode and multithreaded performance mode

The project is built around parity testing: core gameplay-facing behavior is validated across backends so engine code can switch providers without rewriting systems.

## Current State

- **Version line**: `0.2.1-SNAPSHOT` (post-`0.2.0` stabilization)
- **Java target**: Java 25
- **Core status**:
  - Rigid bodies, constraints, characters, vehicles, ragdolls
  - Snapshot/restore support
  - Determinism soak gates
  - Cross-backend parity suite
  - Benchmark regression guard + CI workflow

## Why DynamisPhysics

- One API for simulation features used by gameplay and animation systems
- Deterministic mode for replay/verification workflows
- Native-performance path (Jolt) without giving up backend parity discipline
- Strong local validation workflow (gates + stress + benchmarks)

## Module Layout

| Module | Purpose |
|---|---|
| `dynamisphysics-api` | Public interfaces, descriptors, events, world contracts |
| `dynamisphysics-ode4j` | ODE4J backend implementation |
| `dynamisphysics-jolt` | Jolt backend implementation |
| `dynamisphysics-test` | Shared test harness, scene builders, assertions |
| `dynamisphysics-bench` | JMH benchmark suite |

## Requirements

- Java 25+
- Maven 3.9+
- macOS arm64 for Jolt native flow used in this repository
- Vectrix `1.10.13+`

## Determinism and Threading Model

- **ODE4J deterministic mode**: enabled by config; tuned for repeatable simulation runs.
- **Jolt deterministic mode**: runs with single-thread job system by default.
- **Jolt performance mode**: set `-Djolt.threads=<N>` to enable multithread stepping.

Recommended defaults:

- Validation/parity runs: deterministic mode
- Performance profiling/stress: explicit `-Djolt.threads=8` (or machine-tuned)

## Tuning Profiles (0.3.0)

`PhysicsWorldConfig` now supports profile-driven tuning via `PhysicsTuning`.

Available profiles:

- `DETERMINISTIC`
- `DEFAULT`
- `PERF`

Override precedence (highest wins):

1. System properties
2. Explicit `PhysicsWorldConfig.tuning` fields
3. Profile defaults
4. Backend fallback defaults

Example precedence:

- profile = `PERF`
- config `threads = 4`
- runtime `-Djolt.threads=8`
- resolved result: `threads = 8`

System property overrides:

- `physics.profile=DETERMINISTIC|DEFAULT|PERF`
- `physics.deterministic=true|false`
- `physics.solver.iterations=<int>`
- `jolt.threads=<int>`
- `jolt.alloc=safe|malloc|impl`
- `jolt.alloc.mb=<int>`

## Quick Start (Validation First)

Run these in order from repo root.

### 1) ODE4J gate

```bash
./scripts/gate-ode4j.sh
```

### 2) Jolt parity gate

```bash
./scripts/gate-jolt.sh
```

### 3) Jolt stress gate (optional, high signal)

```bash
./scripts/gate-jolt-stress.sh
```

## Canonical Gate Commands

### ODE4J gate (CI-safe default)

```bash
mvn test -pl dynamisphysics-api,dynamisphysics-test,dynamisphysics-ode4j
```

### Jolt parity gate

```bash
mvn -pl dynamisphysics-jolt -am test -Pjolt-tests \
  -Dtest="JoltNativeSmokeTest,JoltStepSmokeTest,BackendParityTest,JoltCharacterParityTest,JoltVehicleParityTest,JoltRagdollParityTest,SnapshotParityTest,HeavySnapshotParityTest,CompoundDynamicsParityTest,SixDofSpringParityTest,MechanicalConstraintsParityTest,MeshCollisionParityTest" \
  -Dsurefire.failIfNoSpecifiedTests=false
```

### Jolt stress gate

```bash
mvn -pl dynamisphysics-jolt -am test -Pjolt-tests -Djolt.threads=8 \
  -Dphysics.jolt.stress=true -Dphysics.jolt.stress.size=5000 -Dphysics.jolt.stress.islands=200 \
  -Dtest=JoltParallelStabilityTest -Dsurefire.failIfNoSpecifiedTests=false
```

### Long determinism soak (manual/nightly)

```bash
./scripts/gate-long-determinism.sh
```

## Benchmarks

### Build benchmark jar

```bash
mvn -pl dynamisphysics-bench -am package -DskipTests
```

### Smoke run

```bash
java -jar dynamisphysics-bench/target/dynamisphysics-bench.jar -wi 1 -i 1 -f 0 -t 1
```

### Full run

```bash
java -jar dynamisphysics-bench/target/dynamisphysics-bench.jar -wi 3 -i 5 -f 1
```

Bench suites include:

- `RigidBodyStepBenchmark`
- `RaycastBenchmark`
- `VehicleBenchmark`

## Benchmark Regression Guard

### Capture baseline

```bash
./scripts/bench-baseline-capture.sh 0.2.0
```

### Run regression against baseline

```bash
./scripts/bench-regression.sh bench-baselines/0.2.0.json
```

Defaults:

- Throughput fail threshold: `current/baseline < 0.80`
- Time/op fail threshold: `current/baseline > 1.25`
- Jolt threads pinned to `1` unless overridden

CI wrapper:

```bash
./scripts/bench-regression-ci.sh
```

## Development Workflow

Use branch-first flow for all non-trivial work:

1. `feature/<topic>` branch
2. split commits into `feat` and `test/parity`
3. run gates locally
4. merge to `main`

For architecture caveats and intentional deviations, see:

- `ARCHITECTURE_NOTES.md`
- `CHANGELOG.md`

## Troubleshooting

### Jolt native warnings on Java 25

Use native-access flag where relevant:

- `--enable-native-access=ALL-UNNAMED`

This is already wired into Jolt/bench scripts used in this repo.

### Toolchain/JDK mismatch

This repository is Java 25. Ensure Maven and runtime use JDK 25 for normal gates.

### Native crashes

Jolt CI workflows upload crash artifacts (`hs_err_pid*.log`, macOS diagnostics) for postmortem analysis.
