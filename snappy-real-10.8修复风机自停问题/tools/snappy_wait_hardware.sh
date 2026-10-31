#!/usr/bin/env bash
# ============================================================
# @file snappy_wait_hardware.sh
# @brief 开机阶段等待 snappy-real 需要的硬件就绪
# ============================================================

set -u

ROOT="${SNAPPY_ROOT:-/root/snappy-real}"
MODE="${1:-control}"

WAIT_USB="${SNAPPY_BOOT_WAIT_USB:-90}"
WAIT_SERIAL="${SNAPPY_BOOT_WAIT_SERIAL:-15}"
WAIT_CAN="${SNAPPY_BOOT_WAIT_CAN:-15}"
SETTLE="${SNAPPY_BOOT_SETTLE:-8}"

REQUIRE_BOARD="${SNAPPY_REQUIRE_BOARD_PORTS:-0}"
REQUIRE_CAN="${SNAPPY_REQUIRE_CAN:-0}"

# ------------------------------------------------------------
# USB-RS485 三路默认不再作为硬要求（原实现写死 require=1）。
#
# 一个 CH344 通道掉线 / 没插时，硬要求会让 ExecStartPre 失败，
# 配合 Restart=always 与 45s 一次的 boot-guard 就变成"无限重试但永远起不来"
# —— 即"一个模块的失败拖垮整机"。
# 这与配置里已经开启的 allow_partial_devices /
# ignore_device_startup_failures 是矛盾的：配置说"缺设备也启动"，
# 脚本却说"缺一路就别启动"。
#
# 现在默认只告警并继续启动：缺的那一路设备会被设备级退避隔离，
# 只影响它自己（logical device 粒度），同端口与其他端口的设备不受影响。
# 需要恢复旧的严格行为时：SNAPPY_REQUIRE_USB_PORTS=1
# ------------------------------------------------------------
REQUIRE_USB="${SNAPPY_REQUIRE_USB_PORTS:-0}"

USB_PORTS="${SNAPPY_USB_PORTS:-/dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2}"
BOARD_PORTS="${SNAPPY_BOARD_PORTS:-/dev/ttyS0 /dev/ttyS3}"

log() { echo "[snappy-wait] $*" >&2; }

# ------------------------------------------------------------
# 用 /proc/uptime 的单调时钟算截止时间。
# 不用 date +%s：开机时 NTP 校时会把系统时间突然拉偏，
# 用墙上时钟算超时会算错（可能提前超时，也可能永不超时）。
# 如果连 awk 都没有，退化为"用固定轮数"，绝不允许变成无限循环。
# ------------------------------------------------------------
HAVE_AWK=1
command -v awk >/dev/null 2>&1 || HAVE_AWK=0

now_ticks() {
  if [ "$HAVE_AWK" = "1" ]; then
    awk '{print int($1 * 100)}' /proc/uptime 2>/dev/null || echo 0
  else
    echo 0
  fi
}

deadline_from_now() { echo $(( $(now_ticks) + $1 * 100 )); }

# ------------------------------------------------------------
# 1. debugfs 兜底挂载
#    snappy_usb_host_mode.sh 要写 /sys/kernel/debug/usb/*/mode，
#    debugfs 没挂时它会打印 skip missing 然后静默返回 0，
#    USB 就永远停在 device 模式，ttyACM* 永远不出现。
# ------------------------------------------------------------
ensure_debugfs() {
  if grep -q ' /sys/kernel/debug ' /proc/mounts 2>/dev/null; then
    return 0
  fi
  log "debugfs 未挂载，尝试挂载 /sys/kernel/debug"
  if /bin/mount -t debugfs none /sys/kernel/debug 2>/dev/null; then
    log "debugfs 挂载成功"
  else
    log "WARN: debugfs 挂载失败，USB host 模式切换可能被跳过"
  fi
  return 0
}

# ------------------------------------------------------------
# 2. USB 控制器切 host 模式
# ------------------------------------------------------------
usb_host_mode() {
  local script="$ROOT/tools/snappy_usb_host_mode.sh"
  [ -f "$script" ] || { log "WARN: 找不到 $script"; return 0; }
  [ -x "$script" ] || chmod +x "$script" 2>/dev/null || true
  /bin/bash "$script" host >/dev/null 2>&1 || \
    log "WARN: USB host 模式脚本返回非 0（设备节点可能还没到）"
  return 0
}

# ------------------------------------------------------------
# 3. 串口稳定性：连续 3 轮可见（每轮间隔 1s）
#    避开"露了半秒又被拔掉"的 USB 重新枚举中间态
# ------------------------------------------------------------
ports_stable() {
  local ports="$1"
  local round=0 p missing

  while [ "$round" -lt 3 ]; do
    missing=""
    for p in $ports; do
      [ -e "$p" ] || missing="$missing $p"
    done
    if [ -n "$missing" ]; then
      return 1
    fi
    round=$((round + 1))
    if [ "$round" -lt 3 ]; then
      sleep 1
    fi
  done
  return 0
}

missing_of() {
  local ports="$1" p out=""
  for p in $ports; do
    [ -e "$p" ] || out="$out $p"
  done
  echo "$out"
}

# 等待一组串口；等不到时按 require 决定是失败还是告警
wait_ports() {
  local ports="$1" limit="$2" label="$3" retry_host="${4:-0}" require="${5:-1}"
  local deadline round=0

  # 先快速看一眼，已经齐了就不浪费一次 sleep
  if ports_stable "$ports"; then
    log "$label 已就绪:$ports"
    return 0
  fi

  deadline="$(deadline_from_now "$limit")"

  # HAVE_AWK=0 时 now_ticks 恒为 0，deadline 判定会一直成立，
  # 所以额外用轮数上限兜底：每轮约 5s，上限 = limit/5 + 4
  local max_rounds=$(( limit / 5 + 4 ))

  while [ "$(now_ticks)" -lt "$deadline" ] && [ "$round" -lt "$max_rounds" ]; do
    if ports_stable "$ports"; then
      log "$label 已就绪:$ports"
      return 0
    fi

    round=$((round + 1))

    # 每 5 轮重试一次 host 模式切换（mode 切换本身会复位 USB 总线）
    if [ "$retry_host" = "1" ] && [ $((round % 5)) -eq 0 ]; then
      usb_host_mode
    fi
    sleep 2
  done

  if [ "$require" = "1" ]; then
    log "ERROR: $label 在 ${limit}s 内未就绪，缺失:$(missing_of "$ports")"
    return 1
  fi

  log "WARN: $label 在 ${limit}s 内未就绪，缺失:$(missing_of "$ports")，按配置允许设备级失败，继续启动"
  return 0
}

# ------------------------------------------------------------
# 4. can0
# ------------------------------------------------------------
wait_can() {
  local limit="$1"
  local deadline round=0
  local max_rounds=$(( limit + 2 ))

  if [ -e /sys/class/net/can0 ]; then
    log "can0 已就绪"
    return 0
  fi

  deadline="$(deadline_from_now "$limit")"

  while [ "$(now_ticks)" -lt "$deadline" ] && [ "$round" -lt "$max_rounds" ]; do
    if [ -e /sys/class/net/can0 ]; then
      log "can0 已就绪"
      return 0
    fi
    round=$((round + 1))
    sleep 1
  done

  if [ "$REQUIRE_CAN" = "1" ]; then
    log "ERROR: can0 在 ${limit}s 内未出现"
    return 1
  fi
  log "WARN: can0 在 ${limit}s 内未出现，推进风扇可能暂时不可用"
  return 0
}

# ------------------------------------------------------------
# 主流程
# ------------------------------------------------------------
main() {
  log "开始等待硬件 mode=${MODE} root=${ROOT}"

  ensure_debugfs

  if [ "$MODE" = "usb" ]; then
    usb_host_mode
    wait_ports "$USB_PORTS" "$WAIT_USB" "USB-RS485" 1 "$REQUIRE_USB" || return 1
    return 0
  fi

  usb_host_mode

  # 硬要求：USB-RS485 三路
  wait_ports "$USB_PORTS" "$WAIT_USB" "USB-RS485" 1 "$REQUIRE_USB" || return 1

  # 非硬要求：板载串口（默认只告警）
  wait_ports "$BOARD_PORTS" "$WAIT_SERIAL" "板载串口" 0 "$REQUIRE_BOARD" || return 1

  # 非硬要求：can0（默认只告警）
  wait_can "$WAIT_CAN" || return 1

  if [ "$SETTLE" -gt 0 ]; then
    log "等待 ${SETTLE}s 让串口驱动稳定"
    sleep "$SETTLE"
  fi

  # 稳定等待后复核 USB 口，防止这段时间里又发生一次重新枚举
  if ! ports_stable "$USB_PORTS"; then
    log "ERROR: 稳定等待后 USB-RS485 又消失，判定为仍在重新枚举，缺失:$(missing_of "$USB_PORTS")"
    log "HINT: 当前实际枚举: $(ls /dev/serial/by-id/ 2>/dev/null | tr '\n' ' ')"

    if [ "$REQUIRE_USB" = "1" ]; then
      return 1
    fi

    log "WARN: 按配置允许设备级失败，继续启动（缺失通道由设备级退避隔离）"
  fi

  # 板载串口/can0 最终状态只记录，不作为失败条件
  log "最终状态: 板载串口缺失[$(missing_of "$BOARD_PORTS")] can0=$([ -e /sys/class/net/can0 ] && echo present || echo missing)"
  log "硬件就绪检查通过"
  return 0
}

main
exit $?
