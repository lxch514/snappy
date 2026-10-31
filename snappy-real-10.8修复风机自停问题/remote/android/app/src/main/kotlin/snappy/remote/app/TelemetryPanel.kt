/**
 * @file TelemetryPanel.kt
 * @brief 传感器遥测显示面板
 *
 * @details
 * - 按传感器渲染数值与单位。
 * - 过期数据照常显示最后一次有效值，不再标注“过期”。
 * - 无有效数据时字段留空，不显示占位符。
 * - 测距每行两个字段，避免字段被折行拆开。
 */

package snappy.remote.app

import android.widget.TextView
import java.util.Locale
import snappy.protocol.SampleQuality
import snappy.protocol.SensorSampleDto
import snappy.protocol.TelemetryEnvelope

class TelemetryPanel(
    private val view: TextView
) {
    fun render(
        telemetry: TelemetryEnvelope?,
        statusText: String = ""
    ) {
        val samples = telemetry
            ?.samples
            ?.associateBy { it.name }
            .orEmpty()

        view.text = buildString {
            appendLine("测距")

            // 每行最多两个字段。字段较宽（FIELD_WIDTH 含中文字符）时，
            // 四个字段一行会超宽折行，把“机身后：”从中间拆到两行；
            // 主动分行后换行点落在字段边界上。
            appendLine(
                row(
                    field(
                        "毛刷左：",
                        formatDistance(
                            samples["lidar_brush_left_distance"]
                        )
                    ),
                    field(
                        "毛刷右：",
                        formatDistance(
                            samples["lidar_brush_right_distance"]
                        )
                    )
                )
            )

            appendLine(
                row(
                    field(
                        "机身前：",
                        formatDistance(
                            samples["lidar_body_front_distance"]
                        )
                    ),
                    field(
                        "机身后：",
                        formatDistance(
                            samples["lidar_body_rear_distance"]
                        )
                    )
                )
            )

            appendLine("风速")

            appendLine(
                row(
                    field(
                        "机身左：",
                        formatDecimal(
                            samples["anemometer_left_speed"]
                        )
                    ),
                    field(
                        "机身右：",
                        formatDecimal(
                            samples["anemometer_right_speed"]
                        )
                    )
                )
            )

            appendLine("配电柜")

            appendLine(
                row(
                    field(
                        "温度：",
                        formatDecimal(
                            samples["temperature_cabinet_temperature"]
                        )
                    ),
                    field(
                        "湿度：",
                        formatDecimal(
                            samples["temperature_cabinet_humidity"]
                        )
                    )
                )
            )

            appendLine("电源温度：")

            appendLine(
                row(
                    field(
                        "24V电源：",
                        formatDecimal(
                            samples["power_24v_temperature"]
                        )
                    ),
                    field(
                        "48V电源：",
                        formatDecimal(
                            samples["power_48v_temperature"]
                        )
                    )
                )
            )

            appendLine("姿态角")

            append(
                row(
                    field(
                        "横滚角：",
                        formatDecimal(
                            samples["wit_motion_roll"]
                        )
                    ),
                    field(
                        "俯仰角：",
                        formatDecimal(
                            samples["wit_motion_pitch"]
                        )
                    ),
                    field(
                        "航向角：",
                        formatDecimal(
                            samples["wit_motion_yaw"]
                        )
                    )
                )
            )
        }
    }

    private fun row(
        vararg fields: String
    ): String =
        fields.joinToString(separator = "")

    private fun field(
        label: String,
        value: String,
        width: Int = FIELD_WIDTH
    ): String {
        val text = label + value

        return if (text.length >= width) {
            text
        } else {
            text.padEnd(width, ' ')
        }
    }

    private fun formatDistance(
        sample: SensorSampleDto?
    ): String {
        val value =
            displayValue(sample) ?: return ""

        return String.format(
            Locale.US,
            "%.0f mm",
            value
        )
    }

    private fun formatDecimal(
        sample: SensorSampleDto?
    ): String {
        val value =
            displayValue(sample) ?: return ""

        val unit =
            displayUnit(
                sample?.unit?.trim().orEmpty()
            )

        val text = if (unit.isBlank()) {
            String.format(
                Locale.US,
                "%.2f",
                value
            )
        } else {
            String.format(
                Locale.US,
                "%.2f %s",
                value,
                unit
            )
        }

        return text
    }

    private fun displayUnit(
        unit: String
    ): String =
        when (unit) {
            "deg" -> "°"
            "degC" -> "°C"
            else -> unit
        }

    private fun displayValue(
        sample: SensorSampleDto?
    ): Double? {
        val value =
            sample?.value ?: return null

        if (!value.isFinite()) {
            return null
        }

        return when (sample.quality) {
            SampleQuality.GOOD,
            SampleQuality.STALE -> value

            SampleQuality.INVALID,
            SampleQuality.UNKNOWN -> null
        }
    }

    companion object {
        private const val FIELD_WIDTH = 15
    }
}
