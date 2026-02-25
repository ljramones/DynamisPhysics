# DynamisPhysics

Dual-backend physics simulation library for the Dynamis engine ecosystem.
ODE4J and Jolt Physics implementations behind a clean API.

**Status:** 🚧 0.1.0 in development

## Modules

| Module | Purpose |
|--------|---------|
| `dynamisphysics-api` | Zero-dependency interfaces and value types. All consumers depend only on this. |
| `dynamisphysics-ode4j` | ODE4J backend implementation. Pure Java, JPMS-clean. |
| `dynamisphysics-jolt` | Jolt Physics backend implementation. High-performance, multi-threaded. |
| `dynamisphysics-test` | MockPhysicsWorld, PhysicsSimHarness, PhysicsAssertions. |
| `dynamisphysics-bench` | JMH benchmarks — ODE4J vs Jolt head-to-head comparison. |

## Requirements

- Java 25+
- Vectrix 1.10.12

## Architecture Deviations

Backend-specific, documented architecture deviations are tracked in
`ARCHITECTURE_NOTES.md`. Treat that file as authoritative when behavior differs
from the baseline architecture text.

## Gate Commands

### ODE4J gate (default CI-safe)

```bash
mvn test -pl dynamisphysics-api,dynamisphysics-test,dynamisphysics-ode4j
```

### Jolt parity gate

```bash
mvn -pl dynamisphysics-jolt -am test -Pjolt-tests \
  -Dtest="JoltNativeSmokeTest,JoltStepSmokeTest,BackendParityTest,JoltCharacterParityTest,JoltVehicleParityTest,JoltRagdollParityTest,SnapshotParityTest" \
  -Dsurefire.failIfNoSpecifiedTests=false
```

### Jolt stress gate (optional)

```bash
mvn -pl dynamisphysics-jolt -am test -Pjolt-tests -Djolt.threads=8 \
  -Dphysics.jolt.stress=true -Dphysics.jolt.stress.size=5000 -Dphysics.jolt.stress.islands=200 \
  -Dtest=JoltParallelStabilityTest -Dsurefire.failIfNoSpecifiedTests=false
```

### Long determinism soak (manual/nightly)

```bash
./scripts/gate-long-determinism.sh
```

Optional full-scene probe (includes vehicle/ragdoll and ODE4J-only chain constraints):

```bash
mvn -pl dynamisphysics-jolt -am test -Pjolt-tests \
  -Dphysics.long.determinism=true \
  -Dphysics.long.determinism.full=true \
  -Dtest=LongDeterminismSoakTest \
  -Dsurefire.failIfNoSpecifiedTests=false
```

Note: full-scene mode is currently disabled due to a known ODE4J mismatch; use the default soak gate for Block 1 protection.

Convenience scripts:

- `./scripts/gate-ode4j.sh`
- `./scripts/gate-jolt.sh`
- `./scripts/gate-jolt-stress.sh`

## Benchmarks (Step 13)

Build benchmark fat jar:

```bash
mvn -pl dynamisphysics-bench -am package -DskipTests
```

Smoke run:

```bash
java -jar dynamisphysics-bench/target/dynamisphysics-bench.jar -wi 1 -i 1 -f 0 -t 1
```

Full run:

```bash
java -jar dynamisphysics-bench/target/dynamisphysics-bench.jar -wi 3 -i 5 -f 1
```

Included JMH suites:

- `RigidBodyStepBenchmark` (`bodyCount`: 1000, 10000, 50000)
- `RaycastBenchmark` (`raysPerOp`: 1, 100, 1000)
- `VehicleBenchmark` (`vehicleCount`: 10, 100)

Convenience script:

- `./scripts/bench.sh` (`./scripts/bench.sh smoke` for quick run)

## Benchmark Regression Guard

Capture a baseline (stores raw JMH JSON and normalized summary):

```bash
./scripts/bench-baseline-capture.sh 0.1.0
```

Run regression check against a baseline:

```bash
./scripts/bench-regression.sh bench-baselines/0.1.0.json
```

Current defaults:

- Throughput guard: fail if `current/baseline < 0.80`
- Time-per-op guard: fail if `current/baseline > 1.25`
- Jolt threads are pinned to `1` by default (`JOLT_THREADS` override is available)

Tuning knobs:

- `JOLT_THREADS` (default `1`)
- `BENCH_OPS_RATIO_MIN` (default `0.80`)
- `BENCH_TIME_RATIO_MAX` (default `1.25`)
- `BENCH_WI`, `BENCH_I`, `BENCH_FORKS`, `BENCH_THREADS`

Notes:

- `BENCH_FORKS` defaults to `0` for local/sandbox compatibility.
- Use `BENCH_FORKS=1` on dedicated benchmark runners.
- CI wrapper: `./scripts/bench-regression-ci.sh` (defaults: `BENCH_FORKS=1`, `BENCH_WI=2`, `BENCH_I=3`).
