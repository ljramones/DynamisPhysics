#!/usr/bin/env python3
import argparse
import json
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def git_rev() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def normalize_case(entry: dict) -> dict:
    benchmark = entry["benchmark"]
    params = entry.get("params", {}) or {}
    params = dict(sorted(params.items(), key=lambda kv: kv[0]))
    param_key = ",".join(f"{k}={v}" for k, v in params.items())
    key = f"{benchmark}|{param_key}"
    metric = entry["primaryMetric"]
    return {
        "key": key,
        "benchmark": benchmark,
        "params": params,
        "mode": entry.get("mode", ""),
        "score": metric.get("score", 0.0),
        "scoreError": metric.get("scoreError", 0.0),
        "unit": metric.get("scoreUnit", ""),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Extract a normalized baseline summary from JMH JSON")
    parser.add_argument("--input", required=True, help="Raw JMH JSON path")
    parser.add_argument("--output", required=True, help="Output normalized JSON path")
    parser.add_argument("--label", required=True, help="Baseline label, e.g. 0.1.0")
    parser.add_argument("--jolt-threads", default="1", help="Jolt thread count used for the run")
    parser.add_argument("--ops-threshold", type=float, default=0.80, help="Minimum allowed current/baseline ops ratio")
    parser.add_argument("--time-threshold", type=float, default=1.25, help="Maximum allowed current/baseline time ratio")
    args = parser.parse_args()

    raw_path = Path(args.input)
    out_path = Path(args.output)

    entries = json.loads(raw_path.read_text())
    cases = [normalize_case(e) for e in entries]
    cases.sort(key=lambda c: c["key"])

    payload = {
        "schemaVersion": 1,
        "label": args.label,
        "generatedAt": datetime.now(timezone.utc).isoformat(),
        "git": {
            "revision": git_rev(),
        },
        "environment": {
            "python": platform.python_version(),
            "os": platform.system(),
            "arch": platform.machine(),
        },
        "runConfig": {
            "joltThreads": str(args.jolt_threads),
            "opsRatioMin": args.ops_threshold,
            "timeRatioMax": args.time_threshold,
        },
        "cases": cases,
    }

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"Wrote baseline summary: {out_path}")
    print(f"Cases: {len(cases)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
