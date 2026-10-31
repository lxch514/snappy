/**
 * @file NullG20Adapter.kt
 * @brief 空遥控适配器
 *
 * @details
 * - 启动恒失败、状态恒为不可用，用作缺省占位。
 */

package snappy.remote.g20

class NullG20Adapter : G20Adapter {
    override val available: Boolean = false
    override val statusText: String = "G20: unavailable"

    override fun start(listener: (G20ChannelSnapshot) -> Unit): Boolean {
        return false
    }

    override fun stop() {
    }
}