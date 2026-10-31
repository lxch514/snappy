/**
 * @file G20ChannelReader.kt
 * @brief 遥控通道读取接口
 * @details
 * - 只保留启停与通道回调，便于替换底层实现。
 */
 
package snappy.remote.g20

interface G20ChannelReader {
    fun start(listener: (G20ChannelSnapshot) -> Unit): Boolean
    fun stop()
}