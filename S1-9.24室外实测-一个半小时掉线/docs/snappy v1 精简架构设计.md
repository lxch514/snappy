# snappy v1 精简架构设计

> 本文定义目标架构、产品边界和最终验收要求，不表示下述能力已经全部实现。
> 当前代码完成度、可运行范围和明确缺口以 [`architecture.md`](architecture.md) 为准。

## 目标与结论

面向 1–3 台样机，优先保证能启动、可调试、易替换硬件协议。

- RK3588 仍由 `snappy-control` 独占硬件写入和安全决策。
- `snappy-gateway` 同时管理两条机载对外链路：`g20_udp` 和 `ec20_mqtt`。
- G20 UDP（Android 遥控器）是日常操控的主力链路；EC20 MQTT（云端）是辅助/备份链路。
- Android 只通过 G20 UDP 连接 RK3588；完成初始 `control.acquire` 后，日常操作默认由 `g20_udp` 持有 control lease。
- 云端不提供 Web 控制台、HTTP/WSS API、IAM、RBAC 或多设备管理。
- 4G 链路统一使用 MQTT/TLS，不保留 WebSocket。MQTT 已能提供双向通信、心跳、QoS 和 NAT 穿透。
- 云端只保留 Mosquitto、Python 收集进程、SQLite 和人工命令行工具，使用 systemd 部署，不使用 Docker Compose。
- 视频固定采用“RK USB 摄像头 → G20 SDK → Android”链路，不进入 MQTT、云端收集器或 SQLite。这是 v1 及可预见范围内的最终边界，除非后续明确提出新需求。

## 系统拓扑

```text
Android + G20 SDK
        │ UDP/JSON（主力操控）
        ▼
snappy-gateway ── Unix SOCK_SEQPACKET ── snappy-control
        │                                      │
        │                                      ├─ CAN：风扇
        │                                      ├─ RS485：爬绳器电机、继电器、传感器
        │                                      └─ 安全状态机/硬件看门狗
        │ MQTT/TLS over EC20（辅助/备份）
        ▼
Mosquitto ── snappy-cloud-collector ── SQLite/日志文件
        ▲
        └─ snappy-cloudctl（G20 不可用时人工应急接管、查看 ACK）

RK USB 摄像头 ── G20 SDK ── Android
```

视频链路与控制/遥测链路隔离，只走 G20 SDK；不进入 MQTT、云端收集器、SQLite 或其他云端日志存储。

## 目标机载进程职责

### `snappy-control`

- 唯一拥有 CAN、RS485、继电器和安全看门狗设备权限的进程。
- 包含总线驱动、设备抽象、执行器状态机、控制租约、传感器质量检查和安全监督器。
- 所有硬件输出均经过安全状态机，网关不能绕过互锁直接操作执行器。
- 使用 fake CAN/RS485/继电器/传感器实现 x86_64 无硬件测试。

### `snappy-gateway`

- 包含 `G20UdpTransport`、`MqttTransport` 和视频适配器。
- 负责网络认证、JSON 校验、序号/重放检查、心跳和 UDP ACK/重试。
- 将两条链路转换为同一内部命令模型，通过 UDS 交给 `control`。
- `control` 或网关进程死亡时，由 IPC 看门狗和独立硬件安全回路触发安全输出。

## 双链路主次与接管语义

两条链路使用同一命令契约和安全状态机，但职责与使用优先级不同：

- `g20_udp`：Android 遥控器 ↔ RK3588，是日常操作的主力链路；完成初始 `control.acquire` 后，正常操作默认由它持有 lease。
- `ec20_mqtt`：RK3588 ↔ 云端 Mosquitto，是辅助/备份链路；日常不持有 lease，主要用于云端监控以及日志、状态和遥测接收，并在 G20 不可用时允许操作者通过 `snappy-cloudctl` 应急接管。
- 命令 ACK 只沿该命令进入时的已认证链路返回，并引用原 `message_id`；禁止把同一命令 ACK 自动镜像到另一链路。状态、事件、告警、遥测和 presence 可以按发布策略在两条已连接链路上独立上报。
- 普通运动命令只接受当前持有 lease 的链路。
- 非活动链路只允许心跳、诊断、`control.acquire`、停止和急停请求。
- 链路健康检测自动进行，但任何链路失联都不会触发另一条链路自动接管。尤其在主要场景“G20 失联后由 MQTT 应急接管”中，仍必须由操作者通过 `snappy-cloudctl` 明确发送 `control.acquire`。
- 当前链路失联后立即撤销 lease、清空待执行命令并进入 `SAFE_HOLD`。
- 操作者在另一条链路发送 `control.acquire` 后，系统确认安全保持、递增 `control_epoch`，再由新链路重新 `arm`。
- 旧 `session_id`、`lease_id`、`epoch` 或迟到包全部拒绝；恢复连接不会自动续跑。

以下数值只是假链路和首轮实测的候选参数，不是生产默认值；必须优先对主力 G20 UDP 链路进行丢包、抖动、延迟和断连恢复实测，再把经批准的正式值写入部署配置：

- G20 心跳 250 ms；当 G20 持有 lease 时，连续 750 ms 无有效包进入安全保持。
- MQTT 应用心跳 1 s；当 MQTT 持有 lease 时，连续 3 s 无有效续租进入安全保持。
- 网关到控制进程 IPC 心跳 200 ms，600 ms 超时进入安全保持。
- 连续运动命令 TTL 不超过 1 s；抬降动作时长由本机配置限制，不能由远端任意延长。

## 统一 JSON 契约

```json
{
  "schema_version": 1,
  "kind": "command",
  "robot_id": "snappy-001",
  "link_id": "g20_udp",
  "source_id": "remote-01",
  "boot_id": "uuid",
  "session_id": "uuid",
  "message_id": "uuid",
  "seq": 42,
  "issued_at_ms": 0,
  "ttl_ms": 500,
  "control_epoch": 7,
  "lease_id": "uuid",
  "action": "rope.set",
  "params": {
    "direction": "up",
    "speed_pct": 30
  }
}
```

- `link_id` 由网关根据实际入口确定，不能信任客户端自报值。
- ACK 必须引用 `message_id`，并区分 `ACCEPTED`、`RUNNING`、`SUCCEEDED`、`FAILED`、`ABORTED`、`TIMED_OUT`。
- 抬降终态使用 `ASSUMED_REACHED`，不表示真实到位。
- 状态消息必须区分 `desired`、`actual`、`quality`、`age_ms` 和链路健康状态。
- JSON Schema 放在 `contracts/v1/`；只维护 C++、Kotlin、Python 三端的少量 golden test vectors。
- UDS/MQTT 单消息上限 64 KiB；UDP 数据报上限 1200 B，禁止分片。

MQTT 主题：

```text
snappy/v1/robots/{id}/down/command
snappy/v1/robots/{id}/up/ack
snappy/v1/robots/{id}/up/state
snappy/v1/robots/{id}/up/telemetry
snappy/v1/robots/{id}/up/event
snappy/v1/robots/{id}/up/presence
```

命令/ACK/事件使用 QoS 1、`retain=false`；遥测使用 QoS 0；启用 TLS、静态账号、ACL、LWT，禁止离线命令排队。

## 目标安全控制

状态机保留：

```text
BOOT → SELF_TEST → DISARMED → ACQUIRED → ARMED_IDLE → ACTIVE
                         ↘ SAFE_HOLD / FAULT_LATCHED / ESTOP_LATCHED
```

- 姿态越限、超声波无效或陈旧、风速越限、CAN/RS485 故障、网关失联和 lease 过期均由本机处理。
- `SAFE_HOLD` 停止爬绳、滚刷和推杆；继电器失电；风扇仅在独立供电/驱动仍可用且已验证时保持附着转速。
- 外置看门狗、硬件急停、继电器失电断开、方向互锁和最大通电截止不能只依赖 Linux 进程。
- 抬降位置状态为 `UNKNOWN/MOVING_EXTEND/MOVING_RETRACT/ASSUMED_REACHED`，设备健康另用 `GOOD/DEGRADED/FAULT` 表达；换向先断电并留死区，单调时钟强制超时。
- 安全阈值、RS485/CAN 参数或继电器映射缺失时禁止实际 `arm`，但不阻塞 fake 驱动编译。

## 目标云端最小实现

- Mosquitto：仅提供 MQTT/TLS、静态账号、ACL、LWT。
- `snappy-cloud-collector`：Python 单进程，订阅遥测、状态、ACK、事件和 presence，原样写入 SQLite；同时输出 journald/滚动文本日志。
- 云端不接收、转发或存储实时画面；视频数据不发布到 MQTT，也不进入收集器或 SQLite。
- SQLite 最小表：`messages`、`latest_state`；使用 WAL、UTC 时间、按 `robot_id + received_at` 索引和可配置保留期。
- `snappy-cloudctl`：本地/SSH 命令行工具，主要用于 G20 不可用时的人工应急接管；由操作者发布 `control.acquire`、控制命令和心跳，并等待设备 ACK，不做后台调度或自动接管。
- 云端仅使用固定设备密钥/Token，不保存用户体系、不做角色权限、不做审计表。
- 云端不可用时，RK 和 G20 链路仍可独立运行，本机安全逻辑不依赖云端。

## 单仓库目录

```text
snappy/
├── docs/architecture.md
├── contracts/v1/{schemas,test-vectors}
├── rk3588/{control,gateway,drivers,config,tests}
├── remote/android/{app,g20-adapter,protocol}
├── cloud/{broker,collector,cli,schema.sql}
├── simulator/{fake-hardware,fake-g20,fake-mqtt}
├── deploy/{rk3588,cloud-systemd}
└── tools/
```

Android 构建在闭源 G20 AAR 缺失时可以显式选择 `NullG20Adapter`，且该适配器不得模拟网络或设备成功。RK3588 生产组合缺少真实协议、SDK 或必需配置时必须选择 `unconfigured` 后端、保持 fail-closed 并拒绝 `arm`；fake 驱动只允许由测试、模拟器或明确的 fake 组合根注入，禁止 real 初始化失败后在运行时回退为 fake。

## 验收重点

- 两条链路使用同一命令契约；重复、乱序、过期和旧 epoch 命令不会重复执行。
- 活动链路断开、网关/控制进程崩溃、MQTT 重连后均不会自动续跑。
- 重点验证 G20 失联后的行为：系统先撤销 G20 lease 并进入 `SAFE_HOLD`，MQTT 不自动接管；操作者通过 `snappy-cloudctl` 明确 acquire 后，才按 `SAFE_HOLD → acquire → 新 epoch → arm` 恢复人工控制。
- G20 UDP 的心跳与超时参数优先完成实链路压力和故障注入测试，覆盖丢包、抖动、延迟及断连恢复。
- 验证视频只沿“RK USB 摄像头 → G20 SDK → Android”传输，MQTT、云端收集器和 SQLite 均无视频数据入口。
- 推杆方向互锁、单调时钟超时、独立看门狗和失电行为通过故障注入及硬件测试。
- C++、Kotlin、Python 均可在无真实硬件/闭源 SDK 时完成骨架构建和契约测试。
