#!/usr/bin/env bash
set -euo pipefail

BASELINE="${1:-}"
if [[ -z "$BASELINE" ]]; then
  echo "usage: $0 <baseline-json>"
  exit 2
fi
if [[ ! -f "$BASELINE" ]]; then
  echo "baseline not found: $BASELINE"
  exit 2
fi

JOLT_THREADS="${JOLT_THREADS:-1}"
OPS_RATIO_MIN="${BENCH_OPS_RATIO_MIN:-0.80}"
TIME_RATIO_MAX="${BENCH_TIME_RATIO_MAX:-1.25}"

WI="${BENCH_WI:-3}"
ITERS="${BENCH_I:-5}"
FORKS="${BENCH_FORKS:-0}"
THREADS="${BENCH_THREADS:-1}"

PATTERN='org.dynamisphysics.bench.(RigidBodyStepBenchmark|RaycastBenchmark|VehicleBenchmark).*'
RAW="bench-baselines/tmp/current-jmh.json"
CURR="bench-baselines/tmp/current.json"

mkdir -p bench-baselines/tmp

mvn -pl dynamisphysics-bench -am package -DskipTests

java -Djolt.threads="${JOLT_THREADS}" -jar dynamisphysics-bench/target/dynamisphysics-bench.jar \
  -wi "${WI}" -i "${ITERS}" -f "${FORKS}" -t "${THREADS}" \
  -rf json -rff "${RAW}" \
  "${PATTERN}" \
  -p bodyCount=1000,10000 \
  -p raysPerOp=100,1000 \
  -p vehicleCount=10,100

python3 scripts/bench_extract_baseline.py \
  --input "${RAW}" \
  --output "${CURR}" \
  --label "current" \
  --jolt-threads "${JOLT_THREADS}" \
  --ops-threshold "${OPS_RATIO_MIN}" \
  --time-threshold "${TIME_RATIO_MAX}"

python3 scripts/bench_compare.py \
  --baseline "${BASELINE}" \
  --current "${CURR}" \
  --ops-ratio-min "${OPS_RATIO_MIN}" \
  --time-ratio-max "${TIME_RATIO_MAX}"
