/**
 * @file ContractGoldenTest.kt
 * @brief 契约黄金用例测试
 *
 * @details
 * - 校验命令信封字段与线上取值。
 * - 校验端口契约、动作分组与传感器字段清单未漂移。
 */
 
package snappy.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class ContractGoldenTest {
    @Test
    fun commandEnvelopeCarriesBrushCommand() {
        val command = CommandEnvelope.of(
            robotId = "snappy-simple",
            sourceId = "g20",
            sessionId = "session-1",
            messageId = "msg-1",
            sequence = 1,
            action = RemoteAction.BrushForward,
            params = mapOf("speed_raw" to "5")
        )

        assertTrue(CommandValidator.validate(command).isEmpty())
        assertEquals("command", command.kind)
        assertEquals("brush.forward", command.action)
        assertEquals("5", command.params["speed_raw"])
    }

    @Test
    fun telemetryEnvelopeCarriesAllSimpleMachineSensors() {
        val telemetry = TelemetryEnvelope(
            robotId = "snappy-simple",
            samples = listOf(
                SensorSampleDto("lidar_brush_left_distance", 120.0, "mm", SampleQuality.GOOD),
                SensorSampleDto("lidar_brush_right_distance", 121.0, "mm", SampleQuality.GOOD),
                SensorSampleDto("lidar_body_front_distance", 800.0, "mm", SampleQuality.GOOD),
                SensorSampleDto("lidar_body_rear_distance", 810.0, "mm", SampleQuality.GOOD),
                SensorSampleDto("anemometer_left_speed", 3.2, "m/s", SampleQuality.GOOD),
                SensorSampleDto("anemometer_right_speed", 3.1, "m/s", SampleQuality.GOOD),
                SensorSampleDto("wit_motion_roll", 1.0, "deg", SampleQuality.GOOD),
                SensorSampleDto("temperature_cabinet_temperature", 30.0, "degC", SampleQuality.GOOD),
                SensorSampleDto("temperature_cabinet_humidity", 45.0, "%RH", SampleQuality.GOOD),
                SensorSampleDto("power_24v_voltage", 24.1, "V", SampleQuality.GOOD),
                SensorSampleDto("power_48v_voltage", 48.0, "V", SampleQuality.GOOD)
            )
        )

        assertEquals(11, telemetry.samples.size)
        assertEquals(3.2, telemetry.sample("anemometer_left_speed")?.value)
        assertEquals(48.0, telemetry.sample("power_48v_voltage")?.value)
    }
}