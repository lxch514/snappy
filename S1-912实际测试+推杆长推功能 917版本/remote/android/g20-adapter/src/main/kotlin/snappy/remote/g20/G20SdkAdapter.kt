/**
 * @file G20SdkAdapter.kt
 * @brief G20 遥控器 SDK 输入适配和原始通道日志
 *
 * @details
 * - 保持原有 SDK 连接、读取和回调流程不变。
 * - 原始通道写入 logcat，网关端负责写入机器 .log 文件。
 */

package snappy.remote.g20

import android.content.Context
import android.util.Log
import com.skydroid.rcsdk.RCSDKManager
import com.skydroid.rcsdk.SDKManagerCallBack
import com.skydroid.rcsdk.common.error.SkyException
import com.skydroid.rcsdk.utils.RCSDKUtils

class G20SdkAdapter(private val context: Context) : G20Adapter {
    @Volatile
    override var available: Boolean = false
        private set

    @Volatile
    override var statusText: String = "G20: not connected"
        private set

    private var helper: ReadRCButtonHelper? = null
    private var listener: ((G20ChannelSnapshot) -> Unit)? = null
    private var lastLoggedChannels: List<Int>? = null

    override fun start(listener: (G20ChannelSnapshot) -> Unit): Boolean {
        Log.i(TAG, "start requested")
        this.listener = listener
        statusText = "G20: connecting"

        return try {
            RCSDKManager.initSDK(context, object : SDKManagerCallBack {
                override fun onRcConnected() {
                    available = true
                    statusText = "G20: connected ${RCSDKUtils.getDeviceType()}"
                    Log.i(TAG, "G20 connected device=${RCSDKUtils.getDeviceType()}")
                    startReader()
                }

                override fun onRcConnectFail(e: SkyException?) {
                    available = false
                    statusText = "G20: connect failed ${e ?: ""}"
                    Log.e(TAG, "G20 connect failed error=${e ?: ""}")
                }

                override fun onRcDisconnect() {
                    available = false
                    statusText = "G20: disconnected"
                    Log.w(TAG, "G20 disconnected")
                    stopReader()
                }
            })

            RCSDKManager.setMainThreadCallBack(true)
            RCSDKManager.connectToRC()
            true
        } catch (t: Throwable) {
            available = false
            statusText = "G20: init failed $t"
            Log.e(TAG, "G20 init failed", t)
            false
        }
    }

    private fun startReader() {
        if (helper != null) {
            Log.i(TAG, "G20 reader already started")
            return
        }

        try {
            val nextHelper = ReadRCButtonHelper(RCSDKManager.getDeviceType())
            nextHelper.setListener { buttons ->
                val target = listener ?: return@setListener
                val values = buttons?.toList() ?: emptyList()

                /*
                 * 原始通道只写入 Android logcat，不改变送往网关的数组。
                 * 网关端仍会把完整通道数组写入旧机器格式的 .log 文件。
                 */
                val previous = lastLoggedChannels
                if (previous != values) {
                    Log.i(
                        TAG,
                        "G20 channel change previous=${previous ?: "initial"} current=$values"
                    )
                } else {
                    Log.d(TAG, "G20 channels unchanged=$values")
                }
                lastLoggedChannels = values

                target(
                    G20ChannelSnapshot(
                        values = values,
                        deviceName = RCSDKUtils.getDeviceType().toString()
                    )
                )
            }
            nextHelper.start()
            helper = nextHelper
            Log.i(TAG, "G20 reader started")
        } catch (t: Throwable) {
            statusText = "G20: reader failed $t"
            Log.e(TAG, "G20 reader failed", t)
        }
    }

    private fun stopReader() {
        Log.i(TAG, "G20 reader stopping")
        try {
            helper?.stop()
        } catch (_: Throwable) {
        }
        helper = null
        lastLoggedChannels = null
    }

    override fun stop() {
        Log.i(TAG, "stop requested")
        stopReader()

        try {
            RCSDKManager.disconnectRC()
        } catch (_: Throwable) {
        }

        available = false
        statusText = "G20: stopped"
    }

    companion object {
        private const val TAG = "SnappyG20Sdk"
    }
}
