# Release Checklist

## Preconditions

- JDK 25 is installed and active (`.java-version` = `25`).
- Jolt native setup is available for local parity/stress runs.
- Working tree is clean (no unintended changes).

## Required Gates

### ODE4J gate

```bash
./scripts/gate-ode4j.sh
```

### Jolt parity gate

```bash
./scripts/gate-jolt.sh
```

### Jolt stress gate (optional but recommended)

```bash
./scripts/gate-jolt-stress.sh
```

### Benchmark smoke

```bash
./scripts/bench.sh smoke
```

## Release Cut

### 1) Set release version

```bash
mvn versions:set -DnewVersion=0.1.0 -DgenerateBackupPoms=false
```

### 2) Commit release

```bash
git add -A
git commit -m "release: DynamisPhysics 0.1.0"
```

### 3) Tag

```bash
git tag -a v0.1.0 -m "DynamisPhysics 0.1.0"
```

### 4) Publish

Local publish:

```bash
mvn clean install
```

Remote publish (if distribution management is configured):

```bash
mvn clean deploy -DskipTests
```

## Post-release

Bump to next development version:

```bash
mvn versions:set -DnewVersion=0.1.1-SNAPSHOT -DgenerateBackupPoms=false
git add -A
git commit -m "chore: start 0.1.1-SNAPSHOT"
```
