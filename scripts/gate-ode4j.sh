#!/usr/bin/env bash
set -euo pipefail

mvn test -pl dynamisphysics-api,dynamisphysics-test,dynamisphysics-ode4j
