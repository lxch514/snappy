# snappy v1

## 快速验证

```bash
./tools/test-all.sh
./tools/run-fake.sh
```

## 模块边界

- `rk3588/control`：硬件唯一写入者的生命周期、安全保持和控制接口。
- `rk3588/gateway`：G20 UDP/MQTT 传输抽象、入口标记和控制 IPC 抽象。
- `rk3588/drivers`：CAN、RS485、继电器、传感器接口及未配置占位实现。
- `simulator`：fake hardware、fake G20、fake MQTT 和端到端冒烟测试。
- `contracts/v1`：JSON Schema 与 C++/Kotlin/Python 共用 golden vectors。
- `remote/android`：不依赖 Android SDK/G20 AAR 的 Kotlin 协议和适配器骨架。
- `cloud`：SQLite 收集器、`snappy-cloudctl` 骨架及 Mosquitto 配置模板。

详细边界见 [docs/architecture.md](docs/architecture.md)。

## 开发规范

- [设备驱动开发规范](docs/设备驱动开发规范.md)：设备资料权威顺序、总线/协议、逐设备安全约束、配置门禁与 HIL 验收。
- [设备编码规范](docs/设备编码规范.md)：命名、分层、通信、配置、错误、日志、注释、fake/real 和测试要求。
# Snappy 简易机程序

## 1. 项目用途

Snappy 是基于 RK3588 的简易机控制程序，主要完成：

- G20 遥控器接收与按键/摇杆解析；
- 推进风扇 CAN 控制；
- 毛刷滚筒 RS485 控制；
- L7P-750 雷赛爬绳器 RS485 控制；
- 毛刷滚筒推杆、刮条推杆继电器控制；
- 超声波测距；
- 风速采集；
- 温湿度采集；
- WTGAHRS02A 姿态采集；
- 两路 USB 摄像头 RTP/H264 推流；
- Android 遥控端状态、视频和控制交互。


控制程序和网关程序分开运行：

- `snappy-control`：硬件初始化、传感器轮询、执行器控制、安全停止；
- `snappy-gateway`：G20 遥控器、UDP 状态、遥控按键、视频推流；
- Android：视频显示、状态显示和界面按钮控制。

视频链路不进入 MQTT、SQLite 或传感器链路。

## 2. 启动程序

### 控制进程

```bash
./build/rk3588/control/snappy-control \
  --config rk3588/config/hardware.snappy.json \
  --socket /run/snappy/control.sock
  ```
  
### 网关进程

​```
./build/rk3588/gateway/snappy-gateway \
  --config rk3588/config/hardware.snappy.json \
  --socket /run/snappy/control.sock
​```

### 调试配置

​```
./build/rk3588/control/snappy-control \
  --config rk3588/config/hardware.debug.test.json \
  --socket /run/snappy/control.sock
​```

生产模式不能启用：

- `allow_partial_devices`
- `noop_missing_actuators`
- `ignore_device_startup_failures`
- `quarantine_failed_devices`

## 3. 设备和通信参数

| 设备         | 型号                 | 协议                         | 端口           | ID     | 波特率 | 供电   |
| ------------ | -------------------- | ---------------------------- | -------------- | ------ | ------ | ------ |
| 毛刷左侧测距 | UBS1000-C18R-485     | RS485 Modbus RTU             | `/dev/ttyACM0` | `0x23` | 9600   | DC24V  |
| 毛刷右侧测距 | UBS1000-C18R-485     | RS485 Modbus RTU             | `/dev/ttyACM0` | `0x24` | 9600   | DC24V  |
| 机身前测距   | KS114                | RS485                        | `/dev/ttyACM0` | `0xD0` | 9600   | DC24V  |
| 机身后测距   | KS114                | RS485                        | `/dev/ttyACM0` | `0xD2` | 9600   | DC24V  |
| 左侧风速     | YN8588B              | RS485 Modbus RTU             | `/dev/ttyACM1` | `0x31` | 9600   | DC24V  |
| 右侧风速     | YN8588B              | RS485 Modbus RTU             | `/dev/ttyACM1` | `0x32` | 9600   | DC24V  |
| 姿态         | WTGAHRS02A           | RS485 Modbus RTU             | `/dev/ttyACM1` | `0x65` | 9600   | DC24V  |
| 温湿度       | 温湿度模块            | RS485 Modbus RTU             | `/dev/ttyS3`   | `0x33` | 9600   | DC24V  |
| 继电器       | ZS-DIO-R-10A-4-R-485 | RS485 Modbus RTU             | `/dev/ttyACM2` | `0x43` | 9600   | DC24V  |
| 毛刷滚筒     | BD-201S              | RS485 Modbus RTU             | `/dev/ttyACM2` | `0x03` | 9600   | DC24V  |
| 爬绳器       | L7P-750 雷赛         | RS485 Modbus RTU             | `/dev/ttyS0`   | `0x02` | 9600   | AC220V |
| 24V 开关电源 | 24V 3000W            | RS485 Modbus RTU             | `/dev/ttyACM1` | `0x71` | 9600   | AC220V |
| 48V 开关电源 | 48V 3000W            | RS485 Modbus RTU             | `/dev/ttyS3`   | `0x72` | 9600   | AC220V |
| 左推进风扇   | Hobbywing X6 Plus    | Classic CAN 2.0B / HW-UAVCAN | `can0`         | `0x01` | 1M     | DC48V  |
| 右推进风扇   | Hobbywing X6 Plus    | Classic CAN 2.0B / HW-UAVCAN | `can0`         | `0x02` | 1M     | DC48V  |

推进风扇发送源节点为 `0x64`，RawCommand 通道使用 CH1、CH2，原始油门范围为 `0~8191`。

## 4. 主要工作流程

1. `snappy-control` 读取硬件配置。
2. 初始化 CAN、RS485、继电器和传感器服务。
3. 初始化执行器并进入安全停止状态。
4. RS485 传感器由独立总线 worker 轮询。
5. 传感器结果写入缓存。
6. 控制层只读取传感器缓存，不直接阻塞读取设备。
7. `snappy-gateway` 接收 G20 UDP 数据。
8. G20 按键和摇杆转换为控制命令。
9. 控制命令通过控制 socket 发送给 `snappy-control`。
10. 网关独立管理摄像头子进程和视频状态。
11. 急停、租约失效、CAN/RS485 故障时执行安全停止。

## 5. 遥控器功能

### 毛刷
- L1：毛刷正转/停止交替切换，启动速度为最大 15 挡。
- L2：毛刷反转/停止交替切换，启动速度为最大 15 挡。
### 推进风扇
- 右摇杆向上：推进风扇按初始速度启动。
- 右摇杆向左：降低两个风扇速度，可以长推。
- 右摇杆向右：提高两个风扇速度，可以长推。
- 右摇杆向下：停止并失能推进风扇，左右调速失效。
- 再次向上：重新初始化并启动推进风扇。
### 爬绳器
- R1：爬绳器上升/停止交替切换。
- R2：爬绳器下降/停止交替切换。
- 波轮向左：降低爬绳器速度，可以长按。
- 波轮向右：提高爬绳器速度，可以长按。
### 相机
- 相机键：开始/停止视频。
- SPAY：camera1/camera2 交替切换。
### 刮条
- GS：选择毛刷滚筒推杆或刮条推杆。G为刮条推杆，S为毛刷推杆。
- 左摇杆：控制所选推杆的伸出、缩回和时间调整。
先向上推表示就绪，再左右选择对应要操作的秒数，向左减少对应秒数，向右增加对应秒数，都可以长推操作。最后再向上推启动推杆伸出操作。向下为推杆缩回操作。
### 安全停止
- 暂停键：推进风扇、毛刷、爬绳器、推杆全部停止。

当前遥控器通道：

- CH1：右摇杆左右；
- CH2：右摇杆上下；
- CH3：左摇杆上下；
- CH4：左摇杆左右；
- CH5：GS；
- CH7：L2；
- CH8：相机键；
- CH11：L1；
- CH12：SPAY；
- CH13：暂停；
- CH14：波轮；
- CH15：R1；
- CH16：R2。

## 6. 摄像头控制

- camera1：`/dev/video81`；
- camera2：`/dev/video82`；
- 分辨率：640×480；
- 帧率：30 fps；
- 编码：H264 RTP；
- 任意时刻只允许一个摄像头执行采集、编码和推流；
- 相机键启动时默认选择 camera1；
- SPAY 停止当前摄像头并切换到另一个摄像头；
- 相机键停止后释放 V4L2、GStreamer、MPP 和 UDP 资源。

## 7. 安全原则

- 控制租约失效时执行安全停止。
- 暂停键执行安全停止。
- 按下暂停键或者屏幕“安全停止”按钮，都要重新点击“控制权”获得控制租约。
- CAN 发送连续失败后风扇进入故障状态。
- RS485 设备连续失败后进入隔离或降级状态。
- 推杆必须限制最大通电时间。
- L7P 停止时先写零速度，再关闭伺服使能。
- 真实设备初始化失败不能静默回退为 fake。