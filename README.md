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
