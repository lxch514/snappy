#!/usr/bin/env bash
# ============================================================
# @file snappy_usb_host_mode.sh
# @brief TQ3588 双 USB 控制器切到 host 模式
# ============================================================

set -u

MODE="${1:-host}"
STATUS=0

set_usb_mode() {
  local controller="$1"
  local mode_file="/sys/kernel/debug/usb/${controller}/mode"

  if [[ ! -e "$mode_file" ]]; then
    echo "[snappy-usb] skip missing ${mode_file}" >&2
    return 0
  fi

  if [[ ! -w "$mode_file" ]]; then
    echo "[snappy-usb] not writable ${mode_file}; run as root" >&2
    STATUS=1
    return 0
  fi

  if echo "$MODE" > "$mode_file"; then
    echo "[snappy-usb] ${controller} mode=${MODE}"
  else
    echo "[snappy-usb] failed to set ${controller} mode=${MODE}" >&2
    STATUS=1
  fi
}

set_usb_mode "fc000000.usb"
set_usb_mode "fc400000.usb"

exit "$STATUS"