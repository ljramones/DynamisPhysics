#!/usr/bin/env bash
set -euo pipefail

mvn -pl dynamisphysics-jolt -am test -Pjolt-tests \
  -Dtest="JoltNativeSmokeTest,JoltStepSmokeTest,BackendParityTest,JoltCharacterParityTest,JoltVehicleParityTest,JoltRagdollParityTest,SnapshotParityTest" \
  -Dsurefire.failIfNoSpecifiedTests=false
