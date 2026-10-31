package snappy.protocol

data class SensorSampleDto(
    val name: String,
    val value: Double?,
    val unit: String,
    val quality: SampleQuality = SampleQuality.UNKNOWN,
    val timestampMs: Long = 0,
    val ageMs: Long = 0
)