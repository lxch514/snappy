# RK3588 部署说明

本目录保存 TQ3588 上的 systemd 自启动模板。

当前默认是调试整机模式：

- 控制进程：`/snappy/build/rk3588/control/snappy-control`
- 网关进程：`/snappy/build/rk3588/gateway/snappy-gateway`
- 配置文件：`rk3588/config/hardware.debug.test.json`
- 看门狗：通过 `SNAPPY_WATCHDOG_DISABLE=1` 临时关闭
- 摄像头 USB：启动前执行 `/snappy/tools/snappy_usb_host_mode.sh host`

## 安装到板子后执行一次：

```bash
cd /snappy
chmod +x tools/snappy_usb_host_mode.sh
cp deploy/rk3588/snappy-control.service /etc/systemd/system/snappy-control.service
cp deploy/rk3588/snappy-gateway.service /etc/systemd/system/snappy-gateway.service
systemctl daemon-reload
systemctl enable snappy-control.service snappy-gateway.service
systemctl restart snappy-control.service snappy-gateway.service
```

## 查看状态
```
systemctl status snappy-control.service
systemctl status snappy-gateway.service
journalctl -u snappy-control.service -f
journalctl -u snappy-gateway.service -f
```

## 启动顺序：

1. systemd 启动 `snappy-control`；
2. 控制进程创建 `/run/snappy/control.sock`；
3. systemd 启动 `snappy-gateway`；
4. 网关等待 G20 UDP 和 Android 遥控端；
5. 网络稍后就绪不影响进程提前启动。

##  排查顺序：

systemd-analyze critical-chain snappy-control.service
systemd-analyze critical-chain snappy-gateway.service
ip -details -statistics link show can0
ls -l /dev/ttyS0 /dev/ttyS3 /dev/ttyACM0 /dev/ttyACM1
fuser -v /dev/video81 /dev/video82
