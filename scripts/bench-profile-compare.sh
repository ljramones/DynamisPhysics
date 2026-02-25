#!/usr/bin/env bash
set -euo pipefail

FULL="${FULL:-0}"
JOLT_THREADS="${JOLT_THREADS:-1}"
NATIVE_ACCESS="${NATIVE_ACCESS:---enable-native-access=ALL-UNNAMED}"
BENCH_SUITE="${BENCH_SUITE:-core}"

if [[ "${FULL}" == "1" ]]; then
  WI="${BENCH_WI:-3}"
  ITERS="${BENCH_I:-5}"
  FORKS="${BENCH_FORKS:-1}"
else
  WI="${BENCH_WI:-2}"
  ITERS="${BENCH_I:-3}"
  FORKS="${BENCH_FORKS:-1}"
fi

THREADS="${BENCH_THREADS:-1}"
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
TMP_DIR="bench-baselines/tmp/profile-compare"
RAW_DEFAULT="${TMP_DIR}/default-jmh.json"
RAW_PERF="${TMP_DIR}/perf-jmh.json"
SUM_DEFAULT="${TMP_DIR}/default.json"
SUM_PERF="${TMP_DIR}/perf.json"

mkdir -p "${TMP_DIR}"

echo "[bench-profile-compare] suite=${BENCH_SUITE} mode=$([[ "${FULL}" == "1" ]] && echo full || echo quick) wi=${WI} i=${ITERS} forks=${FORKS} threads=${THREADS} jolt.threads=${JOLT_THREADS}"

mvn -pl dynamisphysics-bench -am package -DskipTests

java ${NATIVE_ACCESS} -Dphysics.profile=DEFAULT -Djolt.threads="${JOLT_THREADS}" \
  -jar dynamisphysics-bench/target/dynamisphysics-bench.jar \
  -wi "${WI}" -i "${ITERS}" -f "${FORKS}" -t "${THREADS}" \
  -rf json -rff "${RAW_DEFAULT}" \
  "${PATTERN}" \
  -p bodyCount=1000,10000 \
  -p raysPerOp=100,1000 \
  -p vehicleCount=10,100 \
  -p constraintCount=100,1000 \
  -p constraintTypeMix=SPRING_ONLY,MIXED \
  -p compoundCount=100,1000 \
  -p childrenPerCompound=2,4 \
  -p ragdollCount=1,10 \
  -p constraintModules=10,100

python3 scripts/bench_extract_baseline.py \
  --input "${RAW_DEFAULT}" \
  --output "${SUM_DEFAULT}" \
  --label "profile-default" \
  --jolt-threads "${JOLT_THREADS}" \
  --ops-threshold "0.80" \
  --time-threshold "1.25"

java ${NATIVE_ACCESS} -Dphysics.profile=PERF -Djolt.threads="${JOLT_THREADS}" \
  -jar dynamisphysics-bench/target/dynamisphysics-bench.jar \
  -wi "${WI}" -i "${ITERS}" -f "${FORKS}" -t "${THREADS}" \
  -rf json -rff "${RAW_PERF}" \
  "${PATTERN}" \
  -p bodyCount=1000,10000 \
  -p raysPerOp=100,1000 \
  -p vehicleCount=10,100 \
  -p constraintCount=100,1000 \
  -p constraintTypeMix=SPRING_ONLY,MIXED \
  -p compoundCount=100,1000 \
  -p childrenPerCompound=2,4 \
  -p ragdollCount=1,10 \
  -p constraintModules=10,100

python3 scripts/bench_extract_baseline.py \
  --input "${RAW_PERF}" \
  --output "${SUM_PERF}" \
  --label "profile-perf" \
  --jolt-threads "${JOLT_THREADS}" \
  --ops-threshold "0.80" \
  --time-threshold "1.25"

python3 - << 'PY'
import json
from pathlib import Path

base = json.loads(Path("bench-baselines/tmp/profile-compare/default.json").read_text())
perf = json.loads(Path("bench-baselines/tmp/profile-compare/perf.json").read_text())

b = {c["key"]: c for c in base["cases"] if c["params"].get("backend") == "JOLT"}
p = {c["key"]: c for c in perf["cases"] if c["params"].get("backend") == "JOLT"}

print("\nJolt profile comparison (PERF vs DEFAULT)")
print("key | default | perf | ratio")
for key in sorted(set(b) & set(p)):
    dv = float(b[key]["score"])
    pv = float(p[key]["score"])
    ratio = pv / dv if dv else float("nan")
    print(f"{key} | {dv:.3f} | {pv:.3f} | {ratio:.3f}")

missing = sorted(set(b) ^ set(p))
for key in missing:
    print(f"WARN missing counterpart case: {key}")
PY

echo "\nArtifacts:"
echo "- ${RAW_DEFAULT}"
echo "- ${RAW_PERF}"
echo "- ${SUM_DEFAULT}"
echo "- ${SUM_PERF}"
