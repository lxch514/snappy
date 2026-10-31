#!/bin/sh

LOG_DIR="/root/var/log/snappy"

# 删除修改时间超过 10 天的日志文件
if [ -d "$LOG_DIR" ]; then
    find "$LOG_DIR" \
        -xdev \
        -type f \
        -name "*.log" \
        -mtime +9 \
        -print \
        -exec rm -f -- {} \;
fi

exit 0