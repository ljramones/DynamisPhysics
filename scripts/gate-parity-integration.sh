#!/usr/bin/env bash
set -euo pipefail

mvn -pl dynamisphysics-jolt -am test -Pjolt-tests \
  -Dphysics.parity.integration=true \
  -Dtest="MixedSceneParityTest,TerrainMeshParityTest,SnapshotMidMotionParityTest" \
  -Dsurefire.failIfNoSpecifiedTests=false
