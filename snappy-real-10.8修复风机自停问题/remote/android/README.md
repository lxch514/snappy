# Android/G20 skeleton

# Snappy Android 遥控端

Android 遥控端通过 G20 UDP 与 RK3588 上的 `snappy-real-gateway` 通信。

## 模块

- `app`：实际 Android 界面、按钮、状态显示和视频播放；
- `g20-adapter`：G20 数据适配；
- `protocol`：命令、状态和 ACK 契约。

## 主要功能

- 控制权、清除故障和安全停止；
- 毛刷正转、反转和停止；
- 双推进风扇同步启动、调速和停止；
- L7P-750 爬绳器上升、下降和停止；
- 毛刷滚筒推杆和刮条推杆控制；
- 测距、风速、温湿度和姿态显示；
- camera1/camera2 单路活动视频显示与切换。

两个摄像头不能同时占用编码和传输链路。任意时刻只有当前选中的摄像头采集、编码和推流。

## APP 图标

图标资源放在：

```text
app/src/main/res/mipmap-*/app_icon.png
app/src/main/res/mipmap-*/app_icon_round.png