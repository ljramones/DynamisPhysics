#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def index_cases(path: Path) -> dict:
    payload = json.loads(path.read_text())
    return {c["key"]: c for c in payload.get("cases", [])}


def unit_kind(unit: str) -> str:
    u = (unit or "").lower()
    if "ops/s" in u:
        return "throughput"
    if u.endswith("/op"):
        return "time"
    return "unknown"


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare current JMH summary against a baseline")
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--current", required=True)
    parser.add_argument("--ops-ratio-min", type=float, default=0.80)
    parser.add_argument("--time-ratio-max", type=float, default=1.25)
    args = parser.parse_args()

    baseline_cases = index_cases(Path(args.baseline))
    current_cases = index_cases(Path(args.current))

    failures = 0
    print("Benchmark regression report")
    print(f"- baseline: {args.baseline}")
    print(f"- current : {args.current}")
    print(f"- thresholds: ops_ratio_min={args.ops_ratio_min:.2f}, time_ratio_max={args.time_ratio_max:.2f}")

    for key in sorted(baseline_cases.keys()):
        b = baseline_cases[key]
        c = current_cases.get(key)
        if c is None:
            failures += 1
            print(f"FAIL missing case: {key}")
            continue

        if b["unit"] != c["unit"]:
            failures += 1
            print(f"FAIL unit mismatch: {key} baseline={b['unit']} current={c['unit']}")
            continue

        kind = unit_kind(b["unit"])
        b_score = float(b["score"])
        c_score = float(c["score"])

        if b_score <= 0.0:
            failures += 1
            print(f"FAIL invalid baseline score: {key} score={b_score}")
            continue

        if kind == "throughput":
            ratio = c_score / b_score
            ok = ratio >= args.ops_ratio_min
            status = "PASS" if ok else "FAIL"
            print(f"{status} {key} current={c_score:.3f} baseline={b_score:.3f} ratio={ratio:.3f}")
            if not ok:
                failures += 1
        elif kind == "time":
            ratio = c_score / b_score
            ok = ratio <= args.time_ratio_max
            status = "PASS" if ok else "FAIL"
            print(f"{status} {key} current={c_score:.6f} baseline={b_score:.6f} ratio={ratio:.3f}")
            if not ok:
                failures += 1
        else:
            print(f"WARN unknown unit kind for {key}: {b['unit']} (skipping threshold check)")

    extras = sorted(set(current_cases.keys()) - set(baseline_cases.keys()))
    for key in extras:
        print(f"WARN extra current case not in baseline: {key}")

    if failures > 0:
        print(f"\nResult: FAIL ({failures} regression issue(s))")
        return 1

    print("\nResult: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
