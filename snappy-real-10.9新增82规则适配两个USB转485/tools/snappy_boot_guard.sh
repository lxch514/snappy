#!/usr/bin/env bash
# ============================================================
# @file snappy_boot_guard.sh
# @brief 开机兜底：确保 snappy-real-control / -gateway 处于运行中
# ============================================================

set -u

CONTROL="snappy-real-control.service"
GATEWAY="snappy-real-gateway.service"

log() { echo "[snappy-guard] $*"; }

active_state_of() {
  systemctl show -p ActiveState --value "$1" 2>/dev/null || echo unknown
}

ensure_running() {
  local unit="$1"
  local state
  state="$(active_state_of "$unit")"

  case "$state" in
    active|activating|reloading|deactivating)
      # 正在跑 / 正在启动 / 正在自动重启 —— 交给 systemd，别插手
      return 0
      ;;
    *)
      log "$unit 状态=$state，执行 start"
      systemctl reset-failed "$unit" >/dev/null 2>&1 || true
      systemctl start "$unit" >/dev/null 2>&1 || true
      log "$unit start 后状态=$(active_state_of "$unit")"
      ;;
  esac

  return 0
}

ensure_running "$CONTROL"
ensure_running "$GATEWAY"

exit 0
