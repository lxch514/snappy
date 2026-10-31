/**
 * @file SensorSampleDto.kt
 * @brief 单个传感器样本
 *
 * @details
 * - 数值、单位、数据质量与时间戳。
 * - 数值为空表示本次未取到有效读数。
 */

package snappy.protocol

data class SensorSampleDto(
    val name: String,
    val value: Double?,
    val unit: String,
    val quality: SampleQuality = SampleQuality.UNKNOWN,
    val timestampMs: Long = 0,
    val ageMs: Long = 0
)