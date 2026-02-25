#!/usr/bin/env bash
set -euo pipefail

mvn -pl dynamisphysics-jolt -am test -Pjolt-tests \
  -Dphysics.long.determinism=true \
  -Dtest=LongDeterminismSoakTest \
  -Dsurefire.failIfNoSpecifiedTests=false
