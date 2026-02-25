#!/usr/bin/env bash
set -euo pipefail

THREADS="${JOLT_THREADS:-8}"
SIZE="${PHYSICS_JOLT_STRESS_SIZE:-5000}"
ISLANDS="${PHYSICS_JOLT_STRESS_ISLANDS:-200}"

mvn -pl dynamisphysics-jolt -am test -Pjolt-tests -Djolt.threads="${THREADS}" \
  -Dphysics.jolt.stress=true \
  -Dphysics.jolt.stress.size="${SIZE}" \
  -Dphysics.jolt.stress.islands="${ISLANDS}" \
  -Dtest=JoltParallelStabilityTest -Dsurefire.failIfNoSpecifiedTests=false
