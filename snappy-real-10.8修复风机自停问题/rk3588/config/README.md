# RK3588 配置说明

## 配置文件

- `hardware.debug.test.json`：现场联调配置，允许部分设备失败和临时隔离；
- `hardware.snappy.json`：正式整机配置，设备初始化失败必须明确报错；
- `hardware.example.json`：仅用于 Schema、模拟器和配置结构示例；
- `hardware.schema.json`：硬件配置字段约束。

## 当前物理端口

| 端口 | 用途 | 参数 |
|---|---|---|
| `can0` | 两台 X6 Plus 推进风扇 | 1 Mbps，29 位扩展帧 |
| `/dev/ttyACM0` | 四个测距模块 | 9600，8N1 |
| `/dev/ttyACM1` | 两个风速模块、姿态模块、24V电源 | 9600，8N1 |
| `/dev/ttyACM2` | 继电器、毛刷滚筒电机 BD-201S | 9600，8N1 |
| `/dev/ttyS0` | 爬绳器电机 L7P-750 | 9600，8N1 |
| `/dev/ttyS3` | 温湿度、48V 电源 | 9600，8N1 |

同一串口由 RS485 scheduler 串行访问，禁止多个进程同时打开。

## 调试和正式模式

调试配置允许：

- `allow_partial_devices`
- `noop_missing_actuators`
- `ignore_device_startup_failures`
- `quarantine_failed_devices`
- CAN `allow_start_failure`

正式配置不得启用这些容错项。

## 修改要求

修改端口、站号、波特率、CAN ID、继电器通道或设备 profile 时，必须同步更新：

1. 两份真实硬件配置；
2. `hardware.schema.json`；
3. 根目录 `README.md`；
4. 对应设备驱动文档；
5. 实机测试记录。