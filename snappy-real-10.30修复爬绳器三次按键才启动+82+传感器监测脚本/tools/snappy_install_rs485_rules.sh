#!/bin/bash
# 把 deploy 里的 USB-RS485 稳定名 udev 规则装到 /etc 并 reload，幂等；由 snappy-rs485-udev.service 的 ExecStart 调用。
# 故意不执行 udevadm trigger；任何失败都 exit 0，不拦开机。
# 环境变量：SNAPPY_ROOT / SNAPPY_RS485_RULES_SRC / SNAPPY_RS485_RULES_DST。

set -u

ROOT="${SNAPPY_ROOT:-/root/snappy-real}"
SRC="${SNAPPY_RS485_RULES_SRC:-$ROOT/deploy/rk3588/70-snappy-rs485.rules}"
DST="${SNAPPY_RS485_RULES_DST:-/etc/udev/rules.d/70-snappy-rs485.rules}"

# 第二条规则：屏蔽 ModemManager 与关闭 USB 自动挂起。
SRC2="${SNAPPY_RS485_HYGIENE_SRC:-$ROOT/deploy/rk3588/81-snappy-rs485-usb-hygiene.rules}"
DST2="${SNAPPY_RS485_HYGIENE_DST:-/etc/udev/rules.d/81-snappy-rs485-usb-hygiene.rules}"

log() { echo "[snappy-rs485-rules] $*"; }

if [ ! -f "$SRC" ]; then
    log "WARN: 找不到规则源文件 $SRC，跳过（/dev/snappy/* 需要别的方式维护）"
    exit 0
fi

if [ -f "$DST" ] && cmp -s "$SRC" "$DST"; then
    log "$DST 已是最新，无需安装"
else
    if cp -p "$SRC" "$DST"; then
        chmod 644 "$DST" 2>/dev/null || true
        log "已安装 $DST"
    else
        log "WARN: 安装 $DST 失败，跳过"
        exit 0
    fi
fi

if [ -f "$SRC2" ]; then
    if [ -f "$DST2" ] && cmp -s "$SRC2" "$DST2"; then
        log "$DST2 已是最新，无需安装"
    elif cp -p "$SRC2" "$DST2"; then
        chmod 644 "$DST2" 2>/dev/null || true
        log "已安装 $DST2"
    else
        log "WARN: 安装 $DST2 失败，跳过"
    fi
else
    log "WARN: 找不到规则源文件 $SRC2，跳过"
fi

if command -v udevadm >/dev/null 2>&1; then
    if udevadm control --reload-rules; then
        log "已 reload udev 规则"
    else
        log "WARN: udevadm control --reload-rules 失败"
    fi
else
    log "WARN: 本机没有 udevadm，跳过 reload"
fi

_n="$(grep -c 'SYMLINK+="snappy/rs485_' "$DST" 2>/dev/null || true)"
log "规则里稳定名条数: ${_n:-0}（期望 4）"
if [ "${_n:-0}" != "4" ]; then
    log "WARN: 稳定名条数不是 4，规则内容可能不对，请人工核对 $DST"
fi

exit 0
