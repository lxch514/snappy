# snappy v1 实现边界

产品架构决策以同目录的
[`snappy v1 精简架构设计.md`](snappy%20v1%20精简架构设计.md) 为准。本文件记录
当前代码骨架如何映射该设计，避免把待确认硬件资料误写成实现。

实现设备协议前必须同时遵守 [设备驱动开发规范](设备驱动开发规范.md) 和
[设备编码规范](设备编码规范.md)。

## 当前已实现

- C++17 的 control、gateway、driver 稳定接口和未配置占位后端。
- fake CAN、RS485、继电器、传感器、G20 与 MQTT 内存实现。
- gateway 的入口 `link_id` 覆盖、64 KiB/1200 B 大小边界及内存控制通道。
- v1 JSON Schema、golden vectors 以及 C++、Kotlin、Python 契约检查。
- Python SQLite 收集器和 `snappy-cloudctl` 命令生成/发布接口。
- Mosquitto、systemd 和不代表生产拓扑的 fake-only 配置占位。
- snappy-control 和 snappy-gateway 分进程运行；
- RS485 按物理端口使用 worker；
- 传感器使用缓存；
- 摄像头由 gateway 独立 worker 和独立进程组管理；

## 明确未实现

- EC20 拨号、真实 MQTT 客户端和 TLS 证书发放。
- 执行器业务状态机、安全阈值和自动化控制策略。
- 真实硬件仍需 HIL 验证，代码已实现不等于硬件认证完成。

上述缺口均以 `null` 配置或 `UNCONFIGURED` 状态暴露。fake 模式不读取真实参数，
因此 x86_64 构建和测试不受硬件资料缺失影响。4G 只保留 MQTT/TLS 扩展点；没有
WebSocket 接口。视频不属于 MQTT、云端收集器或 SQLite 的数据模型。
