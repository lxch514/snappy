#!/usr/bin/env bash
# ============================================================
# @file snappy_boot_diag.sh
# @brief 开机自启动问题取证脚本
# ============================================================

set -u

OUT_DIR="${SNAPPY_DIAG_DIR:-/root/snappy-real-diag}"
STAMP="$(date +%Y%m%d-%H%M%S 2>/dev/null || echo unknown)"
OUT="$OUT_DIR/boot-$STAMP.txt"

mkdir -p "$OUT_DIR" 2>/dev/null || true

CONTROL="snappy-real-control.service"
GATEWAY="snappy-real-gateway.service"
GUARD="snappy-real-boot-guard.timer"

# 同时写文件与屏幕
exec > >(tee "$OUT") 2>&1

sec() { echo; echo "==================== $* ===================="; }

run() {
  echo "\$ $*"
  "$@" 2>&1 || echo "  (退出码 $?)"
}

sec "0. 基本信息"
echo "时间        : $(date 2>/dev/null)"
echo "开机时长    : $(cat /proc/uptime 2>/dev/null)"
echo "Boot ID     : $(cat /proc/sys/kernel/random/boot_id 2>/dev/null)"
echo "内核        : $(uname -a 2>/dev/null)"
echo "systemd     : $(systemctl --version 2>/dev/null | head -1)"
echo "本脚本应运行于：手动 restart 之前"

sec "1. 两个单元的 enable / active 状态"
for u in "$CONTROL" "$GATEWAY" "$GUARD"; do
  echo "--- $u"
  echo "  is-enabled : $(systemctl is-enabled "$u" 2>&1)"
  echo "  is-active  : $(systemctl is-active  "$u" 2>&1)"
  systemctl show -p UnitFileState -p LoadState -p ActiveState -p SubState \
                 -p Result -p NRestarts -p InvocationID -p ExecMainStatus \
                 -p ConditionResult -p FragmentPath "$u" 2>&1 | sed 's/^/  /'
done

sec "2. 开机自启软链是否真的存在"
run ls -l /etc/systemd/system/
echo "--- multi-user.target.wants"
run ls -l /etc/systemd/system/multi-user.target.wants/
echo "--- timers.target.wants"
run ls -l /etc/systemd/system/timers.target.wants/
echo "--- 是否被 mask"
for u in "$CONTROL" "$GATEWAY"; do
  echo "  $u -> $(readlink -f /etc/systemd/system/$u 2>/dev/null || echo '(非软链)')"
done

sec "3. 板子上实际生效的单元内容"
run systemctl cat "$CONTROL"
run systemctl cat "$GATEWAY"

sec "4. systemctl status"
run systemctl status "$CONTROL" --no-pager -l
run systemctl status "$GATEWAY" --no-pager -l

sec "5. journal（本次开机，本单元全量尾部）"
run journalctl -b -u "$CONTROL" --no-pager -l -n 200
run journalctl -b -u "$GATEWAY" --no-pager -l -n 200

sec "6. journal（本次开机，含 snappy 关键字的时间线）"
echo "\$ journalctl -b -o short-precise | grep -i snappy"
journalctl -b --no-pager -l -o short-precise 2>&1 | grep -i -E "snappy|ttyACM|ttyS0|ttyS3|can0" | head -200
echo "\$ journalctl -b -o short-precise (最后 80 行，看开机尾声发生了什么)"
journalctl -b --no-pager -l -o short-precise 2>&1 | tail -80

sec "7. 启动链路与耗时"
run systemd-analyze time
run systemd-analyze blame
run systemd-analyze critical-chain "$CONTROL"
run systemd-analyze critical-chain "$GATEWAY"
run systemctl list-dependencies --after "$CONTROL"
run systemctl list-dependencies --after "$GATEWAY"

sec "8. 失败单元"
run systemctl list-units --failed --no-pager
echo "--- 是否有单元处于 activating 挂着"
systemctl list-units --state=activating --no-pager 2>&1 || true

sec "9. 定时器"
run systemctl list-timers --all --no-pager

sec "10. 硬件实际状态"
echo "--- 串口设备"
run ls -l /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyACM3
run ls -l /dev/ttyS0 /dev/ttyS3
echo "--- debugfs"
run grep -w debugfs /proc/mounts
echo "--- can0"
run ls -l /sys/class/net/can0
run ip -details link show can0
echo "--- 二进制与配置是否在位"
run ls -l /root/snappy-real/build/rk3588/control/snappy-real-control
run ls -l /root/snappy-real/build/rk3588/gateway/snappy-gateway
run ls -l /root/snappy-real/rk3588/config/hardware.debug.test.json
echo "--- 运行期目录 / 日志目录"
run ls -ld /run/snappy-real /root/var/log/snappy-real
echo "--- 进程"
run ps -ef
echo "--- 后台服务日志文件（如已生成）"
run ls -lt /root/var/log/snappy-real

sec "11. 内核识别 USB-RS485 / CAN 的情况"
echo "\$ dmesg | grep -i -E 'usb|ttyACM|can|cdc_acm|ch34|wch' (最后 120 行)"
dmesg 2>&1 | grep -i -E "usb|ttyACM|can|cdc_acm|ch34|wch" | tail -120
run lsusb

sec "12. 复现：手动跑一次等待脚本（只读判断，不会改状态）"
echo "注：这一步会花几秒到几十秒，用于确认脚本自身能不能过"
/bin/bash /root/snappy-real/tools/snappy_wait_hardware.sh control
echo "等待脚本退出码 = $?"

sec "结束"
echo "取证文件：$OUT"
echo "请把这个文件发回给工程师。"
