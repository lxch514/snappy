/**
 * @file VideoStateDto.kt
 * @brief 视频状态与地址
 *
 * @details
 * - 描述运行状态、当前摄像头与两路播放地址。
 */
 
package snappy.protocol

data class VideoStateDto(
    val running: Boolean,
    val activeCamera: Int,
    val activeUrl: String,
    val camera1Url: String,
    val camera2Url: String,
    val targetAddress: String = "",
    val camera1Port: Int = 5600,
    val camera2Port: Int = 5602,
    val detail: String = "",
    val updatedAtMs: Long = System.currentTimeMillis()
)