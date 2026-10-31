#!/bin/sh

# ============================================================
# @file snappy-video-cleanup.sh
# @brief 删除超过保留天数的视频录像
#
# 可用环境变量覆盖：
#   SNAPPY_VIDEO_DIR    默认 /root/var/video
#   SNAPPY_VIDEO_KEEP   保留天数，默认 1
# ============================================================

VIDEO_DIR="${SNAPPY_VIDEO_DIR:-/root/var/video}"
KEEP_DAYS="${SNAPPY_VIDEO_KEEP:-1}"

if [ ! -d "$VIDEO_DIR" ]; then
    exit 0
fi

# -mtime +N 表示“修改时间超过 N+1 个整天”。
# 保留 1 天 -> -mtime +0；保留 3 天 -> -mtime +2   
MTIME_DAYS="$((KEEP_DAYS - 1))"

find "$VIDEO_DIR" \
    -xdev \
    -type f \
    \( -name "*.mp4" -o -name "*.jpg" \) \
    -mtime "+${MTIME_DAYS}" \
    -print \
    -delete

# 删除已经没有视频文件的空目录。
find "$VIDEO_DIR" \
    -depth \
    -type d \
    -empty \
    -delete

exit 0
