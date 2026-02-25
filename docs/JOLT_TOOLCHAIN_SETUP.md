# Jolt Toolchain Setup

Jolt JNI parity tests are isolated behind the `jolt-tests` Maven profile and run on JDK 25.

## Runtime Notes

The repository compiles modules with Java 25 class files (`major version 69`).
Running `-Pjolt-tests` with JDK 21 fails with:

`Unsupported class file major version 69`

Use JDK 25 for all Jolt tests.

## Optional Toolchains

If you use Maven toolchains, configure JDK 25 as the selected toolchain for this repo.

## Jolt Threading

Jolt thread count is configurable via `-Djolt.threads`.

Precedence:

1. `-Djolt.threads=<N>` if provided and `N > 0`
2. otherwise if `PhysicsWorldConfig.deterministic == true`, use `1`
3. otherwise use `Runtime.getRuntime().availableProcessors()`

Examples:

```bash
mvn -pl dynamisphysics-jolt -am test -Pjolt-tests -Dtest=BackendParityTest
mvn -pl dynamisphysics-jolt -am test -Pjolt-tests -Djolt.threads=1 -Dtest=BackendParityTest
mvn -pl dynamisphysics-jolt -am test -Pjolt-tests -Djolt.threads=8 -Dtest=BackendParityTest
```

## Commands

Default build (Jolt tests skipped):

```bash
mvn -pl dynamisphysics-jolt -am test
```

Jolt native smoke test:

```bash
mvn -pl dynamisphysics-jolt -am test -Pjolt-tests -Dtest=JoltNativeSmokeTest
```

Full Jolt parity gate:

```bash
mvn -pl dynamisphysics-jolt -am test -Pjolt-tests \
  -Dtest="JoltNativeSmokeTest,JoltStepSmokeTest,BackendParityTest,JoltCharacterParityTest,JoltVehicleParityTest,JoltRagdollParityTest,SnapshotParityTest" \
  -Dsurefire.failIfNoSpecifiedTests=false
```
