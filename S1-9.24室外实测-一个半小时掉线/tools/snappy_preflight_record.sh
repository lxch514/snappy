#!/bin/sh
set -eu

ROOT_DIR="${SNAPPY_ROOT:-/opt/snappy}"
LOG_DIR="${SNAPPY_LOG_DIR:-/var/log/snappy}"
DIAG_DIR="${SNAPPY_DIAG_DIR:-${LOG_DIR}/diagnostics}"
CAPTURE_DIR="${SNAPPY_CAPTURE_DIR:-${LOG_DIR}/captures}"
CORE_DIR="${SNAPPY_CORE_DIR:-/var/coredump/snappy}"
SERVICE_NAME="${SNAPPY_SERVICE_NAME:-snappy-control.service}"

mkdir -p "${DIAG_DIR}" "${CAPTURE_DIR}" "${CORE_DIR}" 2>/dev/null || true
chmod 0755 "${DIAG_DIR}" "${CAPTURE_DIR}" "${CORE_DIR}" 2>/dev/null || true

STAMP="$(date +%Y%m%d-%H%M%S 2>/dev/null || echo unknown-time)"
LOG_FILE="${DIAG_DIR}/preflight-${SERVICE_NAME}-${STAMP}.log"
LATEST_FILE="${DIAG_DIR}/preflight-latest.log"

run_block() {
    title="$1"
    shift
    {
        echo
        echo "===== ${title} ====="
        "$@" 2>&1 || true
    } >> "${LOG_FILE}"
}

{
    echo "===== snappy preflight $(date -Iseconds 2>/dev/null || date) ====="
    echo "service=${SERVICE_NAME}"
    echo "root=${ROOT_DIR}"
    echo "log_dir=${LOG_DIR}"
    echo "diag_dir=${DIAG_DIR}"
    echo "capture_dir=${CAPTURE_DIR}"
    echo "core_dir=${CORE_DIR}"
    echo "user=$(id 2>/dev/null || echo unknown)"
    echo "kernel=$(uname -a 2>/dev/null || echo unknown)"
    echo "boot_id=$(cat /proc/sys/kernel/random/boot_id 2>/dev/null || echo unknown)"
    echo "uptime=$(cat /proc/uptime 2>/dev/null || echo unknown)"
    echo "cmdline=$(cat /proc/cmdline 2>/dev/null || echo unknown)"
} > "${LOG_FILE}"

run_block "environment" env
run_block "systemd service state" systemctl show "${SERVICE_NAME}" -p ActiveState -p SubState -p Result -p NRestarts -p ExecMainStatus -p ExecMainCode
run_block "recent service journal" journalctl -u "${SERVICE_NAME}" -b -n 120 --no-pager
run_block "recent kernel warnings" sh -c "journalctl -k -b -n 160 --no-pager | grep -Ei 'usb|tty|serial|can|video|v4l2|watchdog|panic|oops|segfault|oom|reset|i/o error' || true"
run_block "network addresses" ip addr
run_block "network routes" ip route
run_block "udp sockets" ss -lunp
run_block "process list" ps -ef
run_block "device nodes" sh -c "ls -l /dev/ACM0 /dev/ACM1 /dev/ttyS0 /dev/ttyS3 /dev/video81 /dev/video82 2>/dev/null || true"
run_block "usb devices" sh -c "lsusb 2>/dev/null || true"
run_block "video devices" sh -c "v4l2-ctl --list-devices 2>/dev/null || true"
run_block "can0 link" sh -c "ip -details link show can0 2>/dev/null || true"
run_block "can interfaces" sh -c "ip -details link show type can 2>/dev/null || true"
run_block "disk usage" df -h
run_block "memory" free -h

if [ -w /proc/sys/kernel/core_pattern ]; then
    echo "${CORE_DIR}/core.%e.%p.%t" > /proc/sys/kernel/core_pattern 2>/dev/null || true
fi

if [ -w /proc/sys/kernel/core_uses_pid ]; then
    echo 1 > /proc/sys/kernel/core_uses_pid 2>/dev/null || true
fi

find "${DIAG_DIR}" -type f -name 'preflight-*.log' -mtime +14 -delete 2>/dev/null || true
find "${CAPTURE_DIR}" -type f -mtime +7 -delete 2>/dev/null || true
find "${CORE_DIR}" -type f -name 'core.*' -mtime +14 -delete 2>/dev/null || true

cp "${LOG_FILE}" "${LATEST_FILE}" 2>/dev/null || true
echo "preflight log: ${LOG_FILE}"