#!/bin/sh
# ============================================================
# @file snappy_packet_capture.sh
# @brief 网络与 CAN 抓包取证
#
# 抓取 G20 UDP 端口与 can0 报文，输出到 captures 目录。
# 现场排查通信异常时手动执行，不影响正在运行的服务。
# ============================================================

set -eu

LOG_DIR="${SNAPPY_LOG_DIR:-/var/log/snappy}"
CAPTURE_DIR="${SNAPPY_CAPTURE_DIR:-${LOG_DIR}/captures}"
DURATION="${SNAPPY_CAPTURE_SECONDS:-120}"
IFACE="${SNAPPY_CAPTURE_IFACE:-any}"
CAN_IFACE="${SNAPPY_CAN_IFACE:-can0}"
ACTION="${1:-snapshot}"

mkdir -p "${CAPTURE_DIR}" 2>/dev/null || true

STAMP="$(date +%Y%m%d-%H%M%S 2>/dev/null || echo unknown-time)"
UDP_FILTER="${SNAPPY_UDP_FILTER:-udp port 14550 or udp port 14551 or udp port 14552 or udp port 5600 or udp port 5602}"

snapshot() {
    OUT="${CAPTURE_DIR}/snapshot-${STAMP}.log"
    {
        echo "===== snappy packet snapshot $(date -Iseconds 2>/dev/null || date) ====="
        echo "iface=${IFACE}"
        echo "can_iface=${CAN_IFACE}"
        echo "udp_filter=${UDP_FILTER}"
        echo
        echo "===== ip addr ====="
        ip addr 2>&1 || true
        echo
        echo "===== ip route ====="
        ip route 2>&1 || true
        echo
        echo "===== udp sockets ====="
        ss -lunp 2>&1 || true
        echo
        echo "===== can link ====="
        ip -details link show "${CAN_IFACE}" 2>&1 || true
        echo
        echo "===== recent net/kernel ====="
        journalctl -k -b -n 120 --no-pager 2>/dev/null | grep -Ei 'udp|can|usb|tty|video|v4l2|net|drop|error' || true
    } > "${OUT}"
    echo "snapshot: ${OUT}"
}

capture_udp() {
    OUT="${CAPTURE_DIR}/g20-udp-${STAMP}.pcap"
    if ! command -v tcpdump >/dev/null 2>&1; then
        echo "tcpdump not found"
        return 1
    fi
    echo "capturing udp to ${OUT} for ${DURATION}s"
    timeout "${DURATION}" tcpdump -i "${IFACE}" -s 0 -w "${OUT}" "${UDP_FILTER}"
}

capture_can() {
    OUT="${CAPTURE_DIR}/can-${CAN_IFACE}-${STAMP}.log"
    if ! command -v candump >/dev/null 2>&1; then
        echo "candump not found"
        return 1
    fi
    echo "capturing can to ${OUT} for ${DURATION}s"
    timeout "${DURATION}" candump -L "${CAN_IFACE}" > "${OUT}"
}

case "${ACTION}" in
    snapshot)
        snapshot
        ;;
    udp)
        capture_udp
        ;;
    can)
        capture_can
        ;;
    all)
        snapshot
        capture_udp || true
        capture_can || true
        ;;
    *)
        echo "Usage: $0 [snapshot|udp|can|all]"
        echo "Environment:"
        echo "  SNAPPY_LOG_DIR=/var/log/snappy"
        echo "  SNAPPY_CAPTURE_DIR=/var/log/snappy/captures"
        echo "  SNAPPY_CAPTURE_SECONDS=120"
        echo "  SNAPPY_CAPTURE_IFACE=any"
        echo "  SNAPPY_CAN_IFACE=can0"
        echo "  SNAPPY_UDP_FILTER='udp port 14550 or udp port 14551 or udp port 14552 or udp port 5600 or udp port 5602'"
        exit 2
        ;;
esac