# ============================================================
# @file run-fake.sh
# @brief 在假硬件上运行一次仿真主程序
#
# 不接真实 CAN / RS485，用于快速验证启动与安全停止流程。
# ============================================================

#!/usr/bin/env bash
set -euo pipefail

SNAPPY_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SNAPPY_BUILD_DIR=${SNAPPY_BUILD_DIR:-"$SNAPPY_ROOT/build"}

if [[ ! -x "$SNAPPY_BUILD_DIR/simulator/snappy-sim" ]]; then
  "$SNAPPY_ROOT/tools/configure.sh"
  cmake --build "$SNAPPY_BUILD_DIR" --parallel
fi

exec "$SNAPPY_BUILD_DIR/simulator/snappy-sim"
