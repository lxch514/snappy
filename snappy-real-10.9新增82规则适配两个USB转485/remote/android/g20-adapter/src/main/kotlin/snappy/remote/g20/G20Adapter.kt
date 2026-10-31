/**
 * @file G20Adapter.kt
 * @brief G20 遥控器输入适配接口
 *
 * @details
 * - 定义启停、状态文本和通道快照回调。
 * - 通道快照承载原始舵量与按键值。
 */
 
package snappy.remote.g20

interface G20Adapter {
    val available: Boolean
    val statusText: String

    fun start(listener: (G20ChannelSnapshot) -> Unit): Boolean
    fun stop()

    companion object {
        const val DEFAULT_ROBOT_ID = G20RemoteDefaults.ROBOT_ID
        const val DEFAULT_SOURCE_ID = "g20-remote"
        const val CHANNEL_COUNT = 16
    }
}

data class G20ChannelSnapshot(
    val values: List<Int>,
    val timestampMs: Long = System.currentTimeMillis(),
    val deviceName: String = "g20"
) {
    fun normalizedChannels(): List<Int> {
        if (values.size == G20Adapter.CHANNEL_COUNT) {
            return values
        }

        val fixed = MutableList(G20Adapter.CHANNEL_COUNT) { 1500 }
        values.take(G20Adapter.CHANNEL_COUNT).forEachIndexed { index, value ->
            fixed[index] = value
        }
        return fixed
    }
}