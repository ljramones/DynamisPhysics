#!/usr/bin/env bash
set -euo pipefail

mvn -pl dynamisphysics-jolt -am test -Pjolt-tests \
  -Dphysics.profile=DETERMINISTIC \
  -Djolt.threads=1 \
  -Dphysics.replay.tests=true \
  -Dtest=ReplayStrictRoundTripTest,ReplayBehaviouralRoundTripTest \
  -Dsurefire.failIfNoSpecifiedTests=false
