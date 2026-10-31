#!/usr/bin/env bash

# ============================================================
# @file snappy_brush_watch.sh
# @brief 整机运行时观察毛刷遥控和界面命令结果
#
# 本脚本只读取 snappy-control 日志：
# - 不重启服务
# - 不修改服务配置
# - 不停止任何设备
# ============================================================

set -u

LOG_DIR="${SNAPPY_LOG_DIR:-/var/log/snappy}"
CONTROL_LOG="${LOG_DIR}/snappy-control.log"

echo "正在监视毛刷控制日志"
echo "日志文件: ${CONTROL_LOG}"
echo "按 Ctrl+C 退出"
echo
echo "测试时按键或点击界面按钮，观察下面结果："
echo "  accepted                  命令已经通过执行器服务"
echo "  failed                    驱动器或通信返回失败"
echo "  rejected                  被方向互锁拒绝"
echo "  action=safe_stop          安全停止已经执行"
echo

tail -n 0 -F "${CONTROL_LOG}" 2>/dev/null |
grep --line-buffered -Ei \
    '\[brush\.command\]|brush_motor|interlock|safe_stop|actuator command'