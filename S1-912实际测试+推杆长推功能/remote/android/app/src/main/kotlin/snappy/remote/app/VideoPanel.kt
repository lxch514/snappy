package snappy.remote.app

/**
 * @file VideoPanel.kt
 * @brief G20 屏幕端 RTP/H264 视频播放面板
 *
 * @details
 *
 * - 只在视频 URL / UDP 端口真正变化时重建接收器和解码器。
 * - 避免状态包周期刷新时反复 restart，导致画面频闪和系统资源抖动。
 * - camera1 / camera2 切换时，旧 UDP 接收线程会先失效并退出，再绑定新端口。
 * - 播放器只负责接收 gateway 发来的 RTP/H264，不主动决定摄像头逻辑。
 */

import android.content.Context
import android.media.MediaCodec
import android.media.MediaFormat
import android.net.wifi.WifiManager
import android.os.Build
import android.util.Log
import android.view.Surface
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.net.SocketTimeoutException
import java.nio.ByteBuffer
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong
import snappy.remote.g20.G20RemoteDefaults

class VideoPanel(
    private val anchorView: View?,
    private val statusView: TextView?,
    private val onStatusChanged: (String) -> Unit = {}
) {
    private val running = AtomicBoolean(false)
    private val decoderLock = Object()
    private val receiverLock = Object()
    private val receiverGeneration = AtomicLong(0)
    private val packetCount = AtomicLong(0)
    private val frameCount = AtomicLong(0)

    private var surfaceView: SurfaceView? = null
    private var decoder: MediaCodec? = null
    private var receiverSocket: DatagramSocket? = null
    private var receiverThread: Thread? = null
    private var multicastLock: WifiManager.MulticastLock? = null

    private var currentUrl: String = G20RemoteDefaults.DEFAULT_VIDEO_URL_CAMERA1
    private var currentPort: Int = G20RemoteDefaults.VIDEO_CAMERA1_PORT
    private var ptsUs: Long = 0L
    private var surfaceReady: Boolean = false

    init {
        ensureSurfaceView()
    }

    fun start(url: String) {
        val nextUrl = normalize(url)
        val nextPort = parsePort(nextUrl)

        val alreadyReceiving =
            running.get() &&
                surfaceReady &&
                nextUrl == currentUrl &&
                nextPort == currentPort &&
                receiverThread?.isAlive == true

        if (alreadyReceiving) {
            return
        }

        currentUrl = nextUrl
        currentPort = nextPort

        ensureSurfaceView()

        if (!surfaceReady) {
            running.set(true)
            updateStatus("播放器：等待显示表面 $currentUrl")
            Log.i(TAG, "start requested before surface ready url=$currentUrl port=$currentPort")
            return
        }

        restartDecoderAndReceiver()
    }

    fun switchTo(url: String) {
        val nextUrl = normalize(url)
        val nextPort = parsePort(nextUrl)

        if (nextUrl == currentUrl &&
            nextPort == currentPort &&
            running.get()
        ) {
            return
        }

        running.set(false)
        receiverGeneration.incrementAndGet()
        stopReceiver()
        stopDecoder()

        currentUrl = nextUrl
        currentPort = nextPort
        running.set(true)

        if (surfaceReady) {
            restartDecoderAndReceiver()
        }
    }

    fun stop() {
        running.set(false)
        receiverGeneration.incrementAndGet()
        stopReceiver()
        stopDecoder()
        releaseMulticastLock()
        updateStatus("播放器：已停止")
    }

    fun currentUrl(): String = currentUrl

    fun isRunning(): Boolean = running.get()

    private fun ensureSurfaceView() {
        if (surfaceView != null) return

        val anchor = anchorView ?: return
        val parent = anchor.parent as? ViewGroup ?: return
        val index = parent.indexOfChild(anchor)
        val params = anchor.layoutParams

        anchor.visibility = View.GONE

        val nextSurface = SurfaceView(anchor.context)
        nextSurface.layoutParams = params
        nextSurface.holder.addCallback(object : SurfaceHolder.Callback {
            override fun surfaceCreated(holder: SurfaceHolder) {
                surfaceReady = true
                Log.i(TAG, "surface created")
                if (running.get()) {
                    restartDecoderAndReceiver(holder.surface)
                }
            }

            override fun surfaceChanged(
                holder: SurfaceHolder,
                format: Int,
                width: Int,
                height: Int
            ) {
                Log.i(TAG, "surface changed width=$width height=$height")
            }

            override fun surfaceDestroyed(holder: SurfaceHolder) {
                surfaceReady = false
                Log.i(TAG, "surface destroyed")
                receiverGeneration.incrementAndGet()
                stopReceiver()
                stopDecoder()
            }
        })

        parent.addView(nextSurface, index)
        surfaceView = nextSurface
    }

    private fun restartDecoderAndReceiver() {
        val surface = surfaceView?.holder?.surface
        if (surface == null || !surface.isValid) {
            running.set(true)
            updateStatus("播放器：等待显示表面 $currentUrl")
            Log.i(TAG, "restart postponed, invalid surface url=$currentUrl")
            return
        }

        restartDecoderAndReceiver(surface)
    }

    private fun restartDecoderAndReceiver(surface: Surface) {
        running.set(true)
        packetCount.set(0)
        frameCount.set(0)

        val generation = receiverGeneration.incrementAndGet()

        stopReceiver()
        startDecoder(surface)
        startReceiver(currentPort, generation)

        updateStatus("播放器：接收 RTP/H264 $currentUrl")
    }

    private fun startDecoder(surface: Surface) {
        synchronized(decoderLock) {
            stopDecoderLocked()

            try {
                val format = MediaFormat.createVideoFormat(
                    "video/avc",
                    G20RemoteDefaults.VIDEO_WIDTH,
                    G20RemoteDefaults.VIDEO_HEIGHT
                )

                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                    format.setInteger(MediaFormat.KEY_LOW_LATENCY, 1)
                }

                format.setInteger("priority", 0)

                decoder = MediaCodec.createDecoderByType("video/avc").apply {
                    configure(format, surface, null, 0)
                    start()
                }

                ptsUs = 0L
                Log.i(
                    TAG,
                    "decoder started ${G20RemoteDefaults.VIDEO_WIDTH}x${G20RemoteDefaults.VIDEO_HEIGHT}"
                )
            } catch (t: Throwable) {
                decoder = null
                Log.e(TAG, "decoder start failed", t)
                updateStatus("播放器：解码器启动失败 $t")
            }
        }
    }

    private fun stopDecoder() {
        synchronized(decoderLock) {
            stopDecoderLocked()
        }
    }

    private fun stopDecoderLocked() {
        val current = decoder ?: return
        decoder = null

        try {
            current.stop()
        } catch (_: Throwable) {
        }

        try {
            current.release()
        } catch (_: Throwable) {
        }

        Log.i(TAG, "decoder stopped")
    }

    private fun startReceiver(port: Int, generation: Long) {
        releaseMulticastLock()
        acquireMulticastLock()

        val thread = Thread({
            val depacketizer = RtpH264AccessUnitDepacketizer { frame ->
                if (generation == receiverGeneration.get()) {
                    queueVideoFrame(frame)
                }
            }

            var socket: DatagramSocket? = null

            try {
                socket = DatagramSocket(null).apply {
                    reuseAddress = true
                    broadcast = true
                    soTimeout = 1000
                    receiveBufferSize = 2 * 1024 * 1024
                    bind(InetSocketAddress(port))
                }

                synchronized(receiverLock) {
                    if (!running.get() ||
                        generation != receiverGeneration.get()
                    ) {
                        socket.close()
                        return@Thread
                    }

                    receiverSocket = socket
                }

                Log.i(TAG, "video bind udp 0.0.0.0:$port broadcast=true")

                val buffer = ByteArray(4096)

                while (running.get() &&
                    generation == receiverGeneration.get()
                ) {
                    try {
                        val packet = DatagramPacket(buffer, buffer.size)
                        socket.receive(packet)

                        val count = packetCount.incrementAndGet()
                        if (count == 1L || count % 60L == 0L) {
                            Log.i(
                                TAG,
                                "video rtp packet count=$count from=${packet.address.hostAddress}:${packet.port} size=${packet.length}"
                            )
                        }

                        depacketizer.consume(packet.data, packet.offset, packet.length)
                    } catch (_: SocketTimeoutException) {
                    }
                }
            } catch (t: Throwable) {
                if (running.get() &&
                    generation == receiverGeneration.get()
                ) {
                    Log.e(TAG, "RTP receive error", t)
                    updateStatus("播放器：RTP 接收异常 $t")
                }
            } finally {
                try {
                    socket?.close()
                } catch (_: Throwable) {
                }

                synchronized(receiverLock) {
                    if (receiverSocket === socket) {
                        receiverSocket = null
                    }
                }
            }
        }, "snappy-video-rtp-$port").apply {
            isDaemon = true
            start()
        }

        synchronized(receiverLock) {
            receiverThread = thread
        }
    }

    private fun stopReceiver() {
        val thread: Thread?
        val socket: DatagramSocket?

        synchronized(receiverLock) {
            thread = receiverThread
            socket = receiverSocket
            receiverThread = null
            receiverSocket = null
        }

        try {
            socket?.close()
        } catch (_: Throwable) {
        }

        if (thread != null &&
            thread.isAlive &&
            thread != Thread.currentThread()
        ) {
            try {
                thread.join(200)
            } catch (_: InterruptedException) {
                Thread.currentThread().interrupt()
            }
        }
    }

    private fun queueVideoFrame(frame: ByteArray) {
        if (!running.get()) {
            return
        }

        val frames = frameCount.incrementAndGet()
        if (frames == 1L || frames % 30L == 0L) {
            Log.i(TAG, "video frame count=$frames size=${frame.size}")
            updateStatus("播放器：已收到视频帧 $frames")
        }

        synchronized(decoderLock) {
            val current = decoder ?: return

            val inputIndex = try {
                current.dequeueInputBuffer(0)
            } catch (_: Throwable) {
                return
            }

            if (inputIndex >= 0) {
                val inputBuffer: ByteBuffer = current.getInputBuffer(inputIndex) ?: return
                inputBuffer.clear()

                if (frame.size <= inputBuffer.capacity()) {
                    inputBuffer.put(frame)

                    current.queueInputBuffer(
                        inputIndex,
                        0,
                        frame.size,
                        ptsUs,
                        0
                    )

                    ptsUs += 1_000_000L / G20RemoteDefaults.VIDEO_FPS
                } else {
                    Log.w(
                        TAG,
                        "drop frame too large size=${frame.size} capacity=${inputBuffer.capacity()}"
                    )
                }
            }

            val info = MediaCodec.BufferInfo()

            while (true) {
                val outputIndex = try {
                    current.dequeueOutputBuffer(info, 0)
                } catch (_: Throwable) {
                    return
                }

                if (outputIndex >= 0) {
                    current.releaseOutputBuffer(outputIndex, true)
                } else if (outputIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                    Log.i(TAG, "decoder output format changed")
                    continue
                } else {
                    break
                }
            }
        }
    }

    private fun acquireMulticastLock() {
        val context = anchorView?.context?.applicationContext ?: return

        try {
            val wifiManager =
                context.getSystemService(Context.WIFI_SERVICE) as? WifiManager ?: return

            multicastLock = wifiManager.createMulticastLock("snappy-video-$currentPort").apply {
                setReferenceCounted(false)
                acquire()
            }

            Log.i(TAG, "video multicast lock acquired")
        } catch (t: Throwable) {
            Log.w(TAG, "video multicast lock unavailable: ${t.message}")
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

    private fun normalize(url: String): String {
        return if (url.isBlank()) {
            G20RemoteDefaults.DEFAULT_VIDEO_URL_CAMERA1
        } else {
            url.trim()
        }
    }

    private fun parsePort(url: String): Int {
        val match = Regex(""":(\d+)""").find(url)
        val parsed = match?.groupValues?.getOrNull(1)?.toIntOrNull()
        return parsed ?: G20RemoteDefaults.VIDEO_CAMERA1_PORT
    }

    private fun updateStatus(text: String) {
        statusView?.text = text
        onStatusChanged(text)
    }

    companion object {
        private const val TAG = "SnappyVideoPanel"
    }
}