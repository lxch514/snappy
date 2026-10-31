/**
 * @file RemoteContract.kt
 * @brief 遥控端契约常量
 *
 * @details
 * - 汇总 schema 版本、默认机器人标识、UDP 与视频端口。
 * - 按控制、毛刷、推杆、爬绳器、风机分组列出可用动作。
 * - 传感器字段名与单位也在此集中声明。
 */
 
package snappy.protocol

data class SensorField(
    val name: String,
    val unit: String,
    val description: String
)

data class UdpPortContract(
    val commandPort: Int = 14550,
    val statusPort: Int = 14551,
    val telemetryPort: Int = 14552,
    val videoCamera1Port: Int = 5600,
    val videoCamera2Port: Int = 5602
)

object RemoteContract {
    const val SCHEMA_VERSION = SNAPPY_REMOTE_SCHEMA_VERSION
    const val ROBOT_ID_DEFAULT = "snappy-001"

    val udp = UdpPortContract()

    val controlActions = listOf(
        RemoteAction.ControlAcquire.wireValue,
        RemoteAction.ControlRelease.wireValue,
        RemoteAction.ControlHeartbeat.wireValue,
        RemoteAction.ControlSafeHold.wireValue,
        RemoteAction.ControlEstop.wireValue,
        RemoteAction.FaultClear.wireValue
    )

    val brushActions = listOf(
        RemoteAction.BrushForward.wireValue,
        RemoteAction.BrushReverse.wireValue,
        RemoteAction.BrushStop.wireValue
    )

    val pushActions = listOf(
        RemoteAction.BrushPushExtend.wireValue,
        RemoteAction.BrushPushRetract.wireValue,
        RemoteAction.BrushPushStop.wireValue,
        RemoteAction.ScraperPushExtend.wireValue,
        RemoteAction.ScraperPushRetract.wireValue,
        RemoteAction.ScraperPushStop.wireValue,
        RemoteAction.PushExtend.wireValue,
        RemoteAction.PushRetract.wireValue,
        RemoteAction.PushStop.wireValue
    )

    val fanActions = listOf(
        RemoteAction.FanPrepare.wireValue,
        RemoteAction.FanEnable.wireValue,
        RemoteAction.FanDisable.wireValue,
        RemoteAction.FanLeftSet.wireValue,
        RemoteAction.FanRightSet.wireValue,
        RemoteAction.FanBothSet.wireValue,
        RemoteAction.FanStop.wireValue
    )

    val climberActions = listOf(
        RemoteAction.ClimberAscend.wireValue,
        RemoteAction.ClimberDescend.wireValue,
        RemoteAction.ClimberStop.wireValue
    )

    val videoActions = listOf(
        RemoteAction.VideoStart.wireValue,
        RemoteAction.VideoStop.wireValue,
        RemoteAction.VideoSwitchCamera.wireValue,
        RemoteAction.VideoToggle.wireValue
    )

    val telemetryActions = listOf(
        RemoteAction.TelemetryEnable.wireValue,
        RemoteAction.TelemetryDisable.wireValue
    )

    val sensorFields = listOf(
        SensorField("lidar_brush_left_distance", "mm", "brush-left ultrasonic distance"),
        SensorField("lidar_brush_right_distance", "mm", "brush-right ultrasonic distance"),
        SensorField("lidar_body_front_distance", "mm", "front ultrasonic distance"),
        SensorField("lidar_body_rear_distance", "mm", "rear ultrasonic distance"),
        SensorField("anemometer_left_speed", "m/s", "left wind speed"),
        SensorField("anemometer_right_speed", "m/s", "right wind speed"),
        SensorField("wit_motion_roll", "deg", "body roll"),
        SensorField("wit_motion_pitch", "deg", "body pitch"),
        SensorField("wit_motion_yaw", "deg", "body yaw"),
        SensorField("temperature_cabinet_temperature", "degC", "cabinet temperature"),
        SensorField("temperature_cabinet_humidity", "%RH", "cabinet humidity"),
        SensorField("power_24v_temperature", "degC", "24V power temperature"),
        SensorField("power_48v_temperature", "degC", "48V power temperature"),
        SensorField("power_24v_voltage", "V", "24V output voltage"),
        SensorField("power_24v_current", "A", "24V output current"),
        SensorField("power_24v_power", "W", "24V output power"),
        SensorField("power_48v_voltage", "V", "48V output voltage"),
        SensorField("power_48v_current", "A", "48V output current"),
        SensorField("power_48v_power", "W", "48V output power")
    )

    val allActions: List<String> by lazy {
        RemoteAction.values().map { it.wireValue }
    }
}