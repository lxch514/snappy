#!/usr/bin/env bash
# ============================================================
# @file test-all.sh
# @brief 一键执行 C++ 与 Kotlin 全部测试
#
# 依次完成：配置 -> 构建 -> ctest -> Kotlin 测试。任一环节失败立即退出。
# ============================================================

set -euo pipefail

SNAPPY_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SNAPPY_BUILD_DIR=${SNAPPY_BUILD_DIR:-"$SNAPPY_ROOT/build"}
export SNAPPY_BUILD_DIR

"$SNAPPY_ROOT/tools/configure.sh"
cmake --build "$SNAPPY_BUILD_DIR" --parallel
ctest --test-dir "$SNAPPY_BUILD_DIR" --output-on-failure
"$SNAPPY_ROOT/tools/test-kotlin.sh"
