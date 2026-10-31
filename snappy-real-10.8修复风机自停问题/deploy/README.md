# RK3588 部署说明

本目录保存 TQ3588 上的 systemd 自启动模板。

当前默认是调试整机模式：

- 控制进程：`/root/snappy-real/build/rk3588/control/snappy-real-control`
- 网关进程：`/root/snappy-real/build/rk3588/gateway/snappy-real-gateway`
- 配置文件：`/root/snappy-real/rk3588/config/hardware.debug.test.json`
- 控制 socket：`/run/snappy-real/control.sock`
- 日志目录：`/root/var/log/snappy-real`
- 看门狗：配置里 `watchdog.enabled=false`，单元里再用
  `SNAPPY_WATCHDOG_DISABLE=1` 关闭；要启用看门狗必须两处同时打开
- 摄像头 USB：由 `tools/snappy_wait_hardware.sh` 在启动前切 Host 模式

## 单元文件与用途

| 文件 | 安装名 | 说明 |
| --- | --- | --- |
| `snappy-real-control.service` | 同名 | 控制进程，必须先起 |
| `snappy-real-gateway.service` | 同名 | G20 UDP 网关，依赖控制进程 |
| `snappy-log-cleanup.service` / `.timer` | 同名 | 日志保留 10 天 |
| `snappy-video-cleanup.service` / `.timer` | 同名 | 录像保留 1 天，录像关闭时无副作用 |
| `snappy-packet-capture.service` | 同名 | 可选抓包，默认不开机自启 |

`tools/snappy_wait_hardware.sh` 是控制进程启动前的硬件就绪等待脚本，
单独手动执行可以快速判断开机时到底缺哪个设备。

## 安装到板子后执行一次：

```bash
cd /root/snappy-real
chmod +x tools/snappy_wait_hardware.sh tools/snappy_usb_host_mode.sh
systemctl disable --now snappy-control.service snappy-gateway.service 2>/dev/null || true
rm -f /etc/systemd/system/snappy-control.service /etc/systemd/system/snappy-gateway.service
cp deploy/rk3588/snappy-real-control.service /etc/systemd/system/
cp deploy/rk3588/snappy-real-gateway.service /etc/systemd/system/
cp deploy/rk3588/snappy-log-cleanup.service  /etc/systemd/system/
cp deploy/rk3588/snappy-log-cleanup.timer    /etc/systemd/system/
cp deploy/rk3588/snappy-video-cleanup.service /etc/systemd/system/
cp deploy/rk3588/snappy-video-cleanup.timer   /etc/systemd/system/
systemctl daemon-reload
systemctl reset-failed snappy-real-control.service snappy-real-gateway.service || true

systemctl enable snappy-real-control.service snappy-real-gateway.service
systemctl enable --now snappy-log-cleanup.timer snappy-video-cleanup.timer
systemctl restart snappy-real-control.service
sleep 2
systemctl restart snappy-real-gateway.service
```

## 查看状态
```bash
systemctl status snappy-real-control.service
systemctl status snappy-real-gateway.service
journalctl -u snappy-real-control.service -b -f
journalctl -u snappy-real-gateway.service -b -f
```

## 启动顺序：

1. `systemd-modules-load` / `sys-kernel-debug.mount` / `local-fs.target` 就绪；
2. `snappy-real-control` 的 `ExecStartPre` 运行
   `tools/snappy_wait_hardware.sh control`：
   debugfs 兜底挂载 → USB 切 Host 模式（可重试）→
   等 `ttyACM0/1/2` 连续稳定 → 等 `ttyS0/ttyS3` → 等 `can0`（超时只告警）→ 稳定静默 8s；
3. 控制进程初始化执行器 / 传感器，创建 `/run/snappy-real/control.sock`；
4. 控制进程 active 后 `snappy-real-gateway` 启动，并在 180s 内轮询等待控制 socket；
5. 网关等待 G20 UDP、Android 遥控端和网络就绪。

任一步不满足，只会让**本次启动失败并自动重试**，
不会像旧配置那样撞上 start-limit 后永久停在 failed。

##  排查顺序：

```bash
# 开机整体耗时和关键路径
systemd-analyze blame | head -20
systemd-analyze critical-chain snappy-real-control.service
systemd-analyze critical-chain snappy-real-gateway.service

# 单元到底有没有被 enable / 有没有被 start-limit 打死
systemctl is-enabled snappy-real-control.service snappy-real-gateway.service
systemctl show -p NRestarts -p Result -p ActiveState snappy-real-control.service

# 硬件就绪情况（手动跑一次，直接看到缺什么）
/bin/bash /root/snappy-real/tools/snappy_wait_hardware.sh control; echo "exit=$?"
ls -l /dev/ttyACM* /dev/ttyS0 /dev/ttyS3
ls -l /sys/class/net/can0
cat /proc/mounts | grep debugfs
ip -details -statistics link show can0
fuser -v /dev/video81 /dev/video83
```

## 状态含义速查

| 现象 | 含义 |
| --- | --- |
| `enabled` 但 `inactive (dead)` | 没被拉起：查 `systemctl status` 首行原因 |
| `failed` + `start request repeated too quickly` | 旧配置的 start-limit 打死，已用 `StartLimitIntervalSec=0` 修掉 |
| `activating (start-pre)` 很久 | 正在跑硬件等待脚本，属正常 |
| `active` 但没有 socket | 看 journal 里串口打开和配置加载的错误 |
