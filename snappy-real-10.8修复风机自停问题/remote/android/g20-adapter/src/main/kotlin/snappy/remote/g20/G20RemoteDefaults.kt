/**
 * @file G20RemoteDefaults.kt
 * @brief 遥控端默认参数
 *
 * @details
 * - 默认机器人标识、UDP 命令/状态/遥测端口与视频端口。
 * - 仅供默认值使用，实际取值以界面设置为准。
 */
 
package snappy.remote.g20

object G20RemoteDefaults {
    const val ROBOT_ID = "snappy-001"

    const val UDP_COMMAND_HOST = "192.168.144.255"
    const val UDP_COMMAND_PORT = 14550
    const val UDP_STATUS_PORT = 14551
    const val UDP_TELEMETRY_PORT = 14552

    const val VIDEO_CAMERA1_PORT = 5600
    const val VIDEO_CAMERA2_PORT = 5602

    const val DEFAULT_VIDEO_URL_CAMERA1 = "udp://@:5600"
    const val DEFAULT_VIDEO_URL_CAMERA2 = "udp://@:5602"
    const val DEFAULT_VIDEO_URL = DEFAULT_VIDEO_URL_CAMERA1

    const val VIDEO_WIDTH = 640
    const val VIDEO_HEIGHT = 480
    const val VIDEO_FPS = 30
}