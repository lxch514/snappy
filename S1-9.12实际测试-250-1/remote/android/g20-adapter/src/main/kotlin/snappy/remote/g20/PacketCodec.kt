package snappy.remote.g20

import org.json.JSONArray
import org.json.JSONObject
import snappy.protocol.AckEnvelope
import snappy.protocol.AckStatus
import snappy.protocol.ActuatorStateDto
import snappy.protocol.CommandEnvelope
import snappy.protocol.ControlState
import snappy.protocol.PacketKind
import snappy.protocol.SampleQuality
import snappy.protocol.SensorSampleDto
import snappy.protocol.StatusEnvelope
import snappy.protocol.TelemetryEnvelope
import snappy.protocol.VideoStateDto
import android.util.Log

object PacketCodec {
    private const val TAG = "SnappyPacketCodec"
    fun encodeCommand(command: CommandEnvelope): String {
        val root = JSONObject()
        root.put("schema_version", command.schemaVersion)
        root.put("kind", command.kind)
        root.put("robot_id", command.robotId)
        root.put("link_id", command.linkId)
        root.put("source_id", command.sourceId)
        root.put("session_id", command.sessionId)
        root.put("message_id", command.messageId)
        root.put("seq", command.sequence)
        root.put("issued_at_ms", command.issuedAtMs)
        root.put("ttl_ms", command.ttlMs)
        root.put("control_epoch", command.controlEpoch)

        if (command.leaseId != null) {
            root.put("lease_id", command.leaseId)
        }

        root.put("action", command.action)

        val params = JSONObject()
        for ((key, value) in command.params) {
            params.put(key, value)
        }
        root.put("params", params)

        return root.toString()
    }

    fun decodeAck(payload: String): AckEnvelope? {
        val root = parseObject(payload) ?: return null
        val kind = root.optString("kind", "")

        val hasAck =
            kind == PacketKind.ACK.wireValue ||
                root.has("status")

        if (!hasAck) {
            return null
        }

        val status =
            parseAckStatus(
                root.optString("status", "")
            ) ?: run {
                Log.w(
                    TAG,
                    "drop ACK with unknown status=" +
                        root.optString("status", "")
                )
                return null
            }

        return AckEnvelope(
            schemaVersion = root.optInt("schema_version", 1),
            kind = PacketKind.ACK.wireValue,
            robotId = root.optString("robot_id", ""),
            messageId = root.optString("message_id", ""),
            sequence = root.optLong(
                "seq",
                root.optLong("sequence", 0)
            ),
            status = status,
            detail = root.optString("detail", ""),
            retryable = root.optBoolean(
                "retryable",
                false
            ),
            leaseId = root.optString(
                "lease_id",
                root.optString("leaseId", "")
            ),
            controlEpoch = root.optLong(
                "control_epoch",
                root.optLong("controlEpoch", 0)
            ),
            receivedAtMs = root.optLong(
                "received_at_ms",
                root.optLong(
                    "ack_time_ms",
                    System.currentTimeMillis()
                )
            )
        )
    }

    fun decodeStatus(payload: String): StatusEnvelope? {
        val root = parseObject(payload) ?: return null
        val kind = root.optString("kind", "")

        val looksLikeStatus =
            kind == PacketKind.STATUS.wireValue ||
                root.has("state") ||
                root.has("ready_for_arm") ||
                root.has("readyForArm")

        if (!looksLikeStatus) return null

        val g20 = root.optJSONObject("g20_udp") ?: JSONObject()
        val watchdog = root.optJSONObject("watchdog") ?: JSONObject()
        val video = root.optJSONObject("video") ?: JSONObject()

        val actuators = root.optJSONObject("actuators") ?: JSONObject()
        val brush = actuators.optJSONObject("brush") ?: JSONObject()
        val fan = actuators.optJSONObject("fan") ?: JSONObject()
        val legacyPush = actuators.optJSONObject("push") ?: JSONObject()
        val brushPush = actuators.optJSONObject("brush_push") ?: legacyPush
        val scraperPush = actuators.optJSONObject("scraper_push") ?: JSONObject()
        val climber = actuators.optJSONObject("climber") ?: JSONObject()

        val actuatorState = ActuatorStateDto(
            brushState = brush.optString("state", "stopped"),
            brushSpeedRaw = brush.optInt("speed_raw", brush.optInt("speedRaw", 0)),

            fanActive = fan.optBoolean("active", false),
            fanLeftRaw = fan.optInt("left_raw", fan.optInt("leftRaw", 0)),
            fanRightRaw = fan.optInt("right_raw", fan.optInt("rightRaw", 0)),

            brushPushState = brushPush.optString("state", "stopped"),
            brushPushRemainingMs = brushPush.optLong(
                "remaining_ms",
                brushPush.optLong("remainingMs", 0)
            ),
            brushPushLastDurationMs = brushPush.optLong(
                "last_duration_ms",
                brushPush.optLong("lastDurationMs", 0)
            ),
            brushPushExtensionMm = brushPush.optInt(
                "extension_mm",
                brushPush.optInt("extensionMm", 0)
            ),

            scraperPushState = scraperPush.optString("state", "stopped"),
            scraperPushRemainingMs = scraperPush.optLong(
                "remaining_ms",
                scraperPush.optLong("remainingMs", 0)
            ),
            scraperPushLastDurationMs = scraperPush.optLong(
                "last_duration_ms",
                scraperPush.optLong("lastDurationMs", 0)
            ),
            scraperPushExtensionMm = scraperPush.optInt(
                "extension_mm",
                scraperPush.optInt("extensionMm", 0)
            ),

            pushState = legacyPush.optString(
                "state",
                brushPush.optString("state", "stopped")
            ),
            pushRemainingMs = legacyPush.optLong(
                "remaining_ms",
                brushPush.optLong("remaining_ms", 0)
            ),
            pushLastDurationMs = legacyPush.optLong(
                "last_duration_ms",
                brushPush.optLong("last_duration_ms", 0)
            ),

            climberState = climber.optString("state", "stopped"),
            climberSpeedRpm = parseClimberSpeedRpm(climber)
        )

        return StatusEnvelope(
            schemaVersion = root.optInt("schema_version", 1),
            kind = PacketKind.STATUS.wireValue,
            robotId = root.optString("robot_id", root.optString("robotId", "")),
            state = parseControlState(root.optString("state", "DISARMED")),
            readyForArm = root.optBoolean(
                "ready_for_arm",
                root.optBoolean("readyForArm", false)
            ),
            g20Online = root.optBoolean(
                "g20_online",
                root.optBoolean("g20Online", true)
            ),
            leaseHeld = root.optBoolean(
                "lease_held",
                root.optBoolean("leaseHeld", false)
            ),
            detail = root.optString("detail", ""),
            controlEpoch = root.optLong(
                "control_epoch",
                root.optLong("controlEpoch", 0)
            ),
            leaseId = root.optString("lease_id", root.optString("leaseId", "")),
            leaseOwner = root.optString("lease_owner", root.optString("leaseOwner", "")),
            safetyReason = root.optString(
                "safety_reason",
                root.optString("safetyReason", "")
            ),
            watchdogEnabled = watchdog.optBoolean(
                "enabled",
                root.optBoolean("watchdog_enabled", false)
            ),
            watchdogActive = watchdog.optBoolean(
                "active",
                root.optBoolean("watchdog_active", false)
            ),
            watchdogTimeoutSec = watchdog.optInt(
                "timeout_sec",
                root.optInt("watchdog_timeout_sec", 0)
            ),
            watchdogLastFeedMs = watchdog.optLong(
                "last_feed_ms",
                root.optLong("watchdog_last_feed_ms", 0)
            ),
            watchdogDetail = watchdog.optString(
                "detail",
                root.optString("watchdog_detail", "")
            ),
            commandPort = g20.optInt(
                "command_port",
                root.optInt("command_port", G20RemoteDefaults.UDP_COMMAND_PORT)
            ),
            statusPort = g20.optInt(
                "status_port",
                root.optInt("status_port", G20RemoteDefaults.UDP_STATUS_PORT)
            ),
            telemetryPort = g20.optInt(
                "telemetry_port",
                root.optInt("telemetry_port", G20RemoteDefaults.UDP_TELEMETRY_PORT)
            ),
            videoTargetAddress = video.optString(
                "target_address",
                root.optString("video_target_address", "")
            ),
            videoCamera1Port = video.optInt(
                "camera1_port",
                root.optInt("video_camera1_port", G20RemoteDefaults.VIDEO_CAMERA1_PORT)
            ),
            videoCamera2Port = video.optInt(
                "camera2_port",
                root.optInt("video_camera2_port", G20RemoteDefaults.VIDEO_CAMERA2_PORT)
            ),
            actuators = actuatorState,
            updatedAtMs = root.optLong("updated_at_ms", System.currentTimeMillis())
        )
    }

    fun decodeTelemetry(payload: String): TelemetryEnvelope? {
        val root = parseObject(payload) ?: return null
        val kind = root.optString("kind", "")
        if (kind.isNotEmpty() && kind != PacketKind.TELEMETRY.wireValue) {
            return null
        }

        if (!root.has("samples")) {
            return null
        }

        val samples = mutableListOf<SensorSampleDto>()
        val array = root.optJSONArray("samples") ?: JSONArray()

        for (i in 0 until array.length()) {
            val item = array.optJSONObject(i) ?: continue
            val hasValue = item.has("value") && !item.isNull("value")

            samples += SensorSampleDto(
                name = item.optString("name", item.optString("logical_name", "")),
                value = if (hasValue) item.optDouble("value") else null,
                unit = item.optString("unit", ""),
                quality = parseSampleQuality(item.optString("quality", "UNKNOWN")),
                timestampMs = item.optLong("timestamp_ms", item.optLong("timestampMs", 0)),
                ageMs = item.optLong("age_ms", item.optLong("ageMs", 0))
            )
        }

        return TelemetryEnvelope(
            schemaVersion = root.optInt("schema_version", 1),
            kind = PacketKind.TELEMETRY.wireValue,
            robotId = root.optString("robot_id", root.optString("robotId", "")),
            samples = samples,
            updatedAtMs = root.optLong("updated_at_ms", System.currentTimeMillis())
        )
    }

    fun decodeVideo(payload: String): VideoStateDto? {
        val root = parseObject(payload) ?: return null
        val kind = root.optString("kind", "")
        if (kind != "video") return null

        return VideoStateDto(
            running = root.optBoolean("running", false),
            activeCamera = root.optInt("active_camera", 1),
            activeUrl = root.optString("active_url", ""),
            camera1Url = root.optString(
                "camera1_url",
                G20RemoteDefaults.DEFAULT_VIDEO_URL_CAMERA1
            ),
            camera2Url = root.optString(
                "camera2_url",
                G20RemoteDefaults.DEFAULT_VIDEO_URL_CAMERA2
            ),
            targetAddress = root.optString("target_address", ""),
            camera1Port = root.optInt("camera1_port", G20RemoteDefaults.VIDEO_CAMERA1_PORT),
            camera2Port = root.optInt("camera2_port", G20RemoteDefaults.VIDEO_CAMERA2_PORT),
            detail = root.optString("detail", ""),
            updatedAtMs = root.optLong("updated_at_ms", System.currentTimeMillis())
        )
    }

    private fun parseObject(payload: String): JSONObject? {
        return try {
            JSONObject(payload.trim())
        } catch (_: Throwable) {
            null
        }
    }

    private fun parseAckStatus(value: String): AckStatus? {
        return when (value.trim().uppercase()) {
            "ACCEPTED" -> AckStatus.ACCEPTED
            "RUNNING" -> AckStatus.RUNNING
            "SUCCEEDED",
            "SUCCESS" -> AckStatus.SUCCEEDED
            "FAILED",
            "FAILURE" -> AckStatus.FAILED
            "ABORTED" -> AckStatus.ABORTED
            "TIMED_OUT",
            "TIMEOUT",
            "TIMEDOUT" -> AckStatus.TIMED_OUT
            else -> null
        }
    }

    private fun parseControlState(value: String): ControlState {
        return when (value.trim().lowercase()) {
            "boot" -> ControlState.BOOT
            "self_test", "selftest" -> ControlState.SELF_TEST
            "disarmed" -> ControlState.DISARMED
            "acquired" -> ControlState.ACQUIRED
            "armed_idle", "armedidle" -> ControlState.ARMED_IDLE
            "active" -> ControlState.ACTIVE
            "safe_hold", "safehold" -> ControlState.SAFE_HOLD
            "fault_latched", "faultlatched" -> ControlState.FAULT_LATCHED
            "estop_latched", "estoplatched" -> ControlState.ESTOP_LATCHED
            else -> ControlState.DISARMED
        }
    }

    private fun parseSampleQuality(value: String): SampleQuality {
        return when (value.trim().lowercase()) {
            "good" -> SampleQuality.GOOD
            "stale" -> SampleQuality.STALE
            "invalid" -> SampleQuality.INVALID
            else -> SampleQuality.UNKNOWN
        }
    }

    private fun parseClimberSpeedRpm(climber: JSONObject): Double {
        return climber.optDouble(
            "speed_rpm",
            climber.optDouble("speedRpm", 0.0)
        )
    }
}