/**
 * @file TelemetryEnvelope.kt
 * @brief 遥测信封
 *
 * @details
 * - 承载一组传感器样本与更新时间。
 */
 
package snappy.protocol

data class TelemetryEnvelope(
    val schemaVersion: Int = SNAPPY_REMOTE_SCHEMA_VERSION,
    val kind: String = PacketKind.TELEMETRY.wireValue,
    val robotId: String,
    val samples: List<SensorSampleDto>,
    val updatedAtMs: Long = System.currentTimeMillis()
) {
    fun sample(name: String): SensorSampleDto? = samples.firstOrNull { it.name == name }
}