#!/usr/bin/env bash
set -euo pipefail

SNAPPY_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SNAPPY_BUILD_DIR=${SNAPPY_BUILD_DIR:-"$SNAPPY_ROOT/build"}
export SNAPPY_BUILD_DIR

"$SNAPPY_ROOT/tools/configure.sh"
cmake --build "$SNAPPY_BUILD_DIR" --parallel
ctest --test-dir "$SNAPPY_BUILD_DIR" --output-on-failure
"$SNAPPY_ROOT/tools/test-kotlin.sh"
