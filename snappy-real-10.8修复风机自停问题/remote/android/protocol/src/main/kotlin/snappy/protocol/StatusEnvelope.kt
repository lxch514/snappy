/**
 * @file StatusEnvelope.kt
 * @brief 状态信封与执行器状态
 *
 * @details
 * - 描述毛刷、推杆、风机、爬绳器与控制权状态。
 */
 
package snappy.protocol

data class ActuatorStateDto(
    val brushState: String = "stopped",
    val brushSpeedRaw: Int = 0,

    val fanActive: Boolean = false,
    val fanLeftRaw: Int = 0,
    val fanRightRaw: Int = 0,

    val brushPushState: String = "stopped",
    val brushPushRemainingMs: Long = 0,
    val brushPushLastDurationMs: Long = 0,
    val brushPushExtensionMm: Int = 0,

    val scraperPushState: String = "stopped",
    val scraperPushRemainingMs: Long = 0,
    val scraperPushLastDurationMs: Long = 0,
    val scraperPushExtensionMm: Int = 0,

    val pushState: String = "stopped",
    val pushRemainingMs: Long = 0,
    val pushLastDurationMs: Long = 0,

    val climberState: String = "stopped",
    val climberSpeedRpm: Double = 0.0
)

data class StatusEnvelope(
    val schemaVersion: Int = SNAPPY_REMOTE_SCHEMA_VERSION,
    val kind: String = PacketKind.STATUS.wireValue,
    val robotId: String,
    val state: ControlState,
    val readyForArm: Boolean,
    val g20Online: Boolean,
    val leaseHeld: Boolean,
    val detail: String = "",
    val controlEpoch: Long = 0,
    val leaseId: String = "",
    val leaseOwner: String = "",
    val safetyReason: String = "",
    val watchdogEnabled: Boolean = false,
    val watchdogActive: Boolean = false,
    val watchdogTimeoutSec: Int = 0,
    val watchdogLastFeedMs: Long = 0,
    val watchdogDetail: String = "",
    val commandPort: Int = 14550,
    val statusPort: Int = 14551,
    val telemetryPort: Int = 14552,
    val videoTargetAddress: String = "",
    val videoCamera1Port: Int = 5600,
    val videoCamera2Port: Int = 5602,
    val actuators: ActuatorStateDto = ActuatorStateDto(),
    val updatedAtMs: Long = System.currentTimeMillis()
)