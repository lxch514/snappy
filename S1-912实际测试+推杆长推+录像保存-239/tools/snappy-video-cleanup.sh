#!/bin/sh

# ============================================================
# Snappy 视频录像自动清理
#
# 保留最近 3 天的视频。
#
# 目录：
#   /root/var/video/camera1
#   /root/var/video/camera2
#
# 删除：
#   修改时间超过 3 天的 .mp4
# ============================================================

VIDEO_DIR="/root/var/video"

if [ ! -d "$VIDEO_DIR" ]; then
    exit 0
fi

find "$VIDEO_DIR" \
    -xdev \
    -type f \
    -name "*.mp4" \
    -mtime +2 \
    -print \
    -delete

# 删除已经没有视频文件的空目录。
find "$VIDEO_DIR" \
    -depth \
    -type d \
    -empty \
    -delete

exit 0
