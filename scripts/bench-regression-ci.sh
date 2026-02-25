#!/usr/bin/env bash
set -euo pipefail

BASELINE="${1:-bench-baselines/0.1.0.json}"

export BENCH_FORKS="${BENCH_FORKS:-1}"
export BENCH_WI="${BENCH_WI:-2}"
export BENCH_I="${BENCH_I:-3}"
export BENCH_THREADS="${BENCH_THREADS:-1}"
export JOLT_THREADS="${JOLT_THREADS:-1}"

./scripts/bench-regression.sh "${BASELINE}"
