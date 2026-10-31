#!/usr/bin/env bash
set -euo pipefail

SNAPPY_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SNAPPY_BUILD_DIR=${SNAPPY_BUILD_DIR:-"$SNAPPY_ROOT/build"}

# Fake/test builds stay fail-closed by default.
# Production hardware builds must explicitly set:
# SNAPPY_ENABLE_REAL_DRIVERS=ON
SNAPPY_ENABLE_REAL_DRIVERS=${SNAPPY_ENABLE_REAL_DRIVERS:-OFF}

cmake \
  -S "$SNAPPY_ROOT" \
  -B "$SNAPPY_BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DSNAPPY_BUILD_TESTS=ON \
  -DSNAPPY_ENABLE_REAL_DRIVERS="$SNAPPY_ENABLE_REAL_DRIVERS"