#!/usr/bin/env bash
set -euo pipefail

NATIVE_ACCESS="${NATIVE_ACCESS:---enable-native-access=ALL-UNNAMED}"

mvn -pl dynamisphysics-bench -am package -DskipTests

if [[ "${1:-}" == "smoke" ]]; then
  java ${NATIVE_ACCESS} -jar dynamisphysics-bench/target/dynamisphysics-bench.jar -wi 1 -i 1 -f 0 -t 1
else
  java ${NATIVE_ACCESS} -jar dynamisphysics-bench/target/dynamisphysics-bench.jar -wi 3 -i 5 -f 1
fi
