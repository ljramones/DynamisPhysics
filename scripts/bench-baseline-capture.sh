#!/usr/bin/env bash
set -euo pipefail

NAME="${1:-}"
if [[ -z "$NAME" ]]; then
  echo "usage: $0 <baseline-name>"
  exit 2
fi

JOLT_THREADS="${JOLT_THREADS:-1}"
NATIVE_ACCESS="${NATIVE_ACCESS:---enable-native-access=ALL-UNNAMED}"
OPS_RATIO_MIN="${BENCH_OPS_RATIO_MIN:-0.80}"
TIME_RATIO_MAX="${BENCH_TIME_RATIO_MAX:-1.25}"

WI="${BENCH_WI:-3}"
ITERS="${BENCH_I:-5}"
FORKS="${BENCH_FORKS:-0}"
THREADS="${BENCH_THREADS:-1}"

PATTERN='org.dynamisphysics.bench.(RigidBodyStepBenchmark|RaycastBenchmark|VehicleBenchmark).*'
RAW="bench-baselines/raw/${NAME}-jmh.json"
OUT="bench-baselines/${NAME}.json"

mkdir -p bench-baselines/raw bench-baselines/tmp

mvn -pl dynamisphysics-bench -am package -DskipTests

java ${NATIVE_ACCESS} -Djolt.threads="${JOLT_THREADS}" -jar dynamisphysics-bench/target/dynamisphysics-bench.jar \
  -wi "${WI}" -i "${ITERS}" -f "${FORKS}" -t "${THREADS}" \
  -rf json -rff "${RAW}" \
  "${PATTERN}" \
  -p bodyCount=1000,10000 \
  -p raysPerOp=100,1000 \
  -p vehicleCount=10,100

python3 scripts/bench_extract_baseline.py \
  --input "${RAW}" \
  --output "${OUT}" \
  --label "${NAME}" \
  --jolt-threads "${JOLT_THREADS}" \
  --ops-threshold "${OPS_RATIO_MIN}" \
  --time-threshold "${TIME_RATIO_MAX}"

echo "Baseline written to ${OUT}"
