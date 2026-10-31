/**
 * @file RemoteUiState.kt
 * @brief 遥控界面状态快照
 *
 * @details
 * - 汇总遥控器、链路、设备、命令、安全与遥测的显示文本。
 * - 各字段都有默认值，界面在首帧数据到达前即可渲染。
 */
 
package snappy.remote.app

import snappy.protocol.ActuatorStateDto
import snappy.protocol.StatusEnvelope
import snappy.protocol.TelemetryEnvelope

data class RemoteUiState(
    val rcStatus: String = "G20: connecting",
    val linkStatus: String = "link: waiting",
    val deviceStatus: String = "device: waiting",
    val commandStatus: String = "command: waiting",
    val channelStatus: String = "channels: waiting",
    val safetyStatus: String = "safety: waiting",
    val watchdogStatus: String = "watchdog: disabled",
    val protocolStatus: String = "UDP 14550/14551/14552",
    val telemetryStatus: String = "telemetry: waiting",
    val videoStatus: String = "video: waiting",
    val videoPlayerStatus: String = "player: waiting",
    val videoUrl: String = Settings.DEFAULT_VIDEO_URL,
    val actuators: ActuatorStateDto = ActuatorStateDto(),
    val status: StatusEnvelope? = null,
    val telemetry: TelemetryEnvelope? = null
)