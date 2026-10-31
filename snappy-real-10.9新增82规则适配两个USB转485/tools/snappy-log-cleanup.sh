#!/bin/sh

# ============================================================
# @file snappy-log-cleanup.sh
# @brief 删除超过保留天数的 snappy-real 日志
#
# 可用环境变量覆盖：
#   SNAPPY_LOG_DIR    默认 /root/var/log/snappy-real
#   SNAPPY_LOG_KEEP   保留天数（整数天），默认 10
# ============================================================

LOG_DIR="${SNAPPY_LOG_DIR:-/root/var/log/snappy-real}"
KEEP_DAYS="${SNAPPY_LOG_KEEP:-10}"

MTIME_DAYS="$((KEEP_DAYS - 1))"

if [ -d "$LOG_DIR" ]; then
    find "$LOG_DIR" \
        -xdev \
        -type f \
        -name "*.log" \
        -mtime "+${MTIME_DAYS}" \
        -print \
        -exec rm -f -- {} \;
fi

exit 0