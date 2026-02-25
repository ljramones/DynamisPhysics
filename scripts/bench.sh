#!/usr/bin/env bash
set -euo pipefail

NATIVE_ACCESS="${NATIVE_ACCESS:---enable-native-access=ALL-UNNAMED}"
BENCH_SUITE="${BENCH_SUITE:-core}"

CORE_PATTERN='org.dynamisphysics.bench.(RigidBodyStepBenchmark|RaycastBenchmark|VehicleBenchmark).*'
EXPANDED_PATTERN='org.dynamisphysics.bench.(ConstraintSolveBenchmark|CompoundPileBenchmark|MixedSceneBenchmark).*'

case "${BENCH_SUITE}" in
  core) PATTERN="${CORE_PATTERN}" ;;
  expanded) PATTERN="${EXPANDED_PATTERN}" ;;
  all) PATTERN="${CORE_PATTERN}|${EXPANDED_PATTERN}" ;;
  *)
    echo "Unknown BENCH_SUITE=${BENCH_SUITE}. Use core|expanded|all."
    exit 2
    ;;
esac

mvn -pl dynamisphysics-bench -am package -DskipTests

if [[ "${1:-}" == "smoke" ]]; then
  java ${NATIVE_ACCESS} -jar dynamisphysics-bench/target/dynamisphysics-bench.jar -wi 1 -i 1 -f 0 -t 1 "${PATTERN}"
else
  java ${NATIVE_ACCESS} -jar dynamisphysics-bench/target/dynamisphysics-bench.jar -wi 3 -i 5 -f 1 "${PATTERN}"
fi
