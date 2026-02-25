#!/usr/bin/env bash
set -euo pipefail

mvn -pl dynamisphysics-bench -am package -DskipTests

if [[ "${1:-}" == "smoke" ]]; then
  java -jar dynamisphysics-bench/target/dynamisphysics-bench.jar -wi 1 -i 1 -f 0 -t 1
else
  java -jar dynamisphysics-bench/target/dynamisphysics-bench.jar -wi 3 -i 5 -f 1
fi
