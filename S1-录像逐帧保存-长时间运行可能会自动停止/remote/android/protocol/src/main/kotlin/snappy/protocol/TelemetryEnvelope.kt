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