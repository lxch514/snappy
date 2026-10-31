#!/usr/bin/env bash
set -euo pipefail

SNAPPY_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SNAPPY_BUILD_DIR=${SNAPPY_BUILD_DIR:-"$SNAPPY_ROOT/build"}

if [[ ! -x "$SNAPPY_BUILD_DIR/simulator/snappy-sim" ]]; then
  "$SNAPPY_ROOT/tools/configure.sh"
  cmake --build "$SNAPPY_BUILD_DIR" --parallel
fi

exec "$SNAPPY_BUILD_DIR/simulator/snappy-sim"
