#!/bin/sh
# ============================================================
# @file snappy_link_watch.sh
# @brief 记录 eth0（G20 链路）载波跳变与内核上下文（只读观测）
#
# ============================================================

set -u

LOG_DIR="${SNAPPY_LOG_DIR:-/root/var/log/snappy-real}"
IFACE="${SNAPPY_LINK_WATCH_IFACE:-eth0}"
MODE="${SNAPPY_LINK_WATCH_MODE:-auto}"
POLL_MS="${SNAPPY_LINK_WATCH_POLL_MS:-1000}"
LOG="${SNAPPY_LINK_WATCH_LOG:-${LOG_DIR}/link-watch.log}"

mkdir -p "$LOG_DIR" 2>/dev/null || true

ts() {
    date '+%Y-%m-%d %H:%M:%S'
}

log() {
    printf '%s [link-watch] %s\n' "$(ts)" "$*" >>"$LOG"
}

# 采集当前链路快照。全部是只读操作。
describe() {
    _iface="$1"

    _carrier="$(cat "/sys/class/net/${_iface}/carrier" 2>/dev/null)" || _carrier="?"
    _state="$(cat "/sys/class/net/${_iface}/operstate" 2>/dev/null)" || _state="?"
    _speed="$(cat "/sys/class/net/${_iface}/speed" 2>/dev/null)" || _speed="?"
    _addr="$(ip -o -4 addr show dev "${_iface}" 2>/dev/null | sed -n '1p' | awk '{print $4}')"

    printf 'carrier=%s operstate=%s speed=%s addr=%s' \
        "${_carrier}" "${_state}" "${_speed}" "${_addr:-none}"
}

run_event() {
    ip monitor link dev "$IFACE" 2>/dev/null |
        while IFS= read -r line; do
            log "event ${IFACE}: ${line} | $(describe "$IFACE")"
        done
}

run_poll() {
    # 亚秒级采样没有意义，这里强制最小 1s。
    _sleep_s=$((POLL_MS / 1000))
    if [ "$_sleep_s" -lt 1 ]; then
        _sleep_s=1
    fi

    _last=""

    while :; do
        _cur="$(describe "$IFACE")"

        if [ "$_cur" != "$_last" ]; then
            log "change ${IFACE}: ${_cur}"
            _last="$_cur"
        fi

        sleep "$_sleep_s"
    done
}

log "watcher started (iface=${IFACE} mode=${MODE} poll_ms=${POLL_MS})"
log "baseline ${IFACE}: $(describe "$IFACE")"

case "$MODE" in
    event)
        run_event
        ;;
    poll)
        run_poll
        ;;
    *)
        # auto：先试事件驱动；若它立刻返回（ip 不支持 monitor 过滤），
        # 说明事件通道不可用，退回低频轮询。
        _start="$(date +%s)"
        run_event
        _elapsed=$(( $(date +%s) - _start ))

        if [ "$_elapsed" -lt 3 ]; then
            log "event mode unavailable or exited early (${_elapsed}s); falling back to poll"
            run_poll
        else
            log "event stream ended after ${_elapsed}s"
        fi
        ;;
esac
