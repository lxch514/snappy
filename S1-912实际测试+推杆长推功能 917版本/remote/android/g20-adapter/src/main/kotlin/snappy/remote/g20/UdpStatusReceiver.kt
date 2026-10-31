package snappy.remote.g20

import android.content.Context
import android.net.wifi.WifiManager
import android.util.Log
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.net.SocketTimeoutException
import java.nio.charset.StandardCharsets
import snappy.protocol.AckEnvelope
import snappy.protocol.StatusEnvelope
import snappy.protocol.VideoStateDto

class UdpStatusReceiver(
    private val listenPort: Int = G20RemoteDefaults.UDP_STATUS_PORT,
    private val context: Context? = null
) {
    interface Listener {
        fun onAck(ack: AckEnvelope) = Unit
        fun onStatus(status: StatusEnvelope) = Unit
        fun onVideo(video: VideoStateDto) = Unit
        fun onRawStatus(payload: String) = Unit
        fun onStatusError(error: String) = Unit
    }

    @Volatile
    private var running = false

    private var socket: DatagramSocket? = null
    private var thread: Thread? = null
    private var multicastLock: WifiManager.MulticastLock? = null

    fun start(listener: Listener): Boolean {
        if (running) {
            Log.i(TAG, "status receiver already running port=$listenPort")
            return true
        }

        return try {
            acquireMulticastLock()

            val receiveSocket = DatagramSocket(null).apply {
                reuseAddress = true
                broadcast = true
                soTimeout = 500
                receiveBufferSize = 1024 * 1024
                bind(InetSocketAddress(listenPort))
            }

            socket = receiveSocket
            running = true

            thread = Thread(
                { receiveLoop(receiveSocket, listener) },
                "snappy-status-rx"
            ).apply {
                isDaemon = true
                start()
            }

            Log.i(TAG, "status bind udp 0.0.0.0:$listenPort broadcast=true")
            true
        } catch (t: Throwable) {
            Log.e(TAG, "status bind failed port=$listenPort", t)
            listener.onStatusError(t.toString())
            releaseMulticastLock()
            false
        }
    }

    private fun receiveLoop(receiveSocket: DatagramSocket, listener: Listener) {
        val buffer = ByteArray(65535)

        while (running) {
            try {
                val packet = DatagramPacket(buffer, buffer.size)
                receiveSocket.receive(packet)

                val payload = String(
                    packet.data,
                    packet.offset,
                    packet.length,
                    StandardCharsets.UTF_8
                )

                Log.i(
                    TAG,
                    "status raw from=${packet.address.hostAddress}:${packet.port} size=${packet.length} ${payload.take(220)}"
                )

                val video = PacketCodec.decodeVideo(payload)
                if (video != null) {
                    listener.onVideo(video)
                    continue
                }

                val ack = PacketCodec.decodeAck(payload)
                if (ack != null) {
                    listener.onAck(ack)
                    continue
                }

                val status = PacketCodec.decodeStatus(payload)
                if (status != null) {
                    listener.onStatus(status)
                    continue
                }

                listener.onRawStatus(payload)
            } catch (_: SocketTimeoutException) {
            } catch (t: Throwable) {
                if (running) {
                    Log.e(TAG, "status receive error", t)
                    listener.onStatusError(t.toString())
                }
            }
        }
    }

    fun stop() {
        running = false

        try {
            socket?.close()
        } catch (_: Throwable) {
        }

        socket = null
        thread = null
        releaseMulticastLock()

        Log.i(TAG, "status receiver stopped port=$listenPort")
    }

    private fun acquireMulticastLock() {
        val appContext = context?.applicationContext ?: return

        try {
            val wifiManager =
                appContext.getSystemService(Context.WIFI_SERVICE) as? WifiManager ?: return

            multicastLock = wifiManager.createMulticastLock("snappy-status-$listenPort").apply {
                setReferenceCounted(false)
                acquire()
            }

            Log.i(TAG, "status multicast lock acquired")
        } catch (t: Throwable) {
            Log.w(TAG, "status multicast lock unavailable: ${t.message}")
        }
    }

    private fun releaseMulticastLock() {
        try {
            multicastLock?.let {
                if (it.isHeld) {
                    it.release()
                }
            }
        } catch (_: Throwable) {
        }

        multicastLock = null
    }

    companion object {
        private const val TAG = "SnappyStatusRx"
    }
}