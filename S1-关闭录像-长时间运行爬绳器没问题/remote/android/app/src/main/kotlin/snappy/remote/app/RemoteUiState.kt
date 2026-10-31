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