/**
 * @file UdpCommandTransport.kt
 * @brief 命令 UDP 收发与可靠投递
 *
 * @details
 * - 普通通道只发最新值；可靠事件单独占道并等待应答。
 * - 等待应答期间保活不受影响，避免被误判为链路中断。
 * - 重试到上限后释放阻塞，不重放已下发过的动作。
 */


package snappy.remote.transport

import android.os.SystemClock
import android.util.Log
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.ArrayDeque
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong
import org.json.JSONArray
import org.json.JSONObject
import snappy.protocol.AckStatus
import snappy.protocol.CommandEnvelope
import snappy.remote.g20.PacketCodec

class UdpCommandTransport(
    host: String,
    port: Int
) {
    private data class Target(
        val host: String,
        val port: Int
    )

    private data class Packet(
        val messageId: String,
        val sequence: Long,
        val payload: String,
        val query: String,
        val queuedAt: Long,
        val reliable: Boolean
    )

    private class Flight(
        val packet: Packet,
        val startedAt: Long
    ) {
        var sends = 0
        var nextAt = startedAt
        var queryOnly = false
        var timedOut = false
    }

    private data class Heartbeat(
        val robotId: String,
        val sourceId: String,
        val sessionId: String,
        val inputAt: Long
    )

    private val closed =
        AtomicBoolean(false)

    @Volatile
    private var target =
        Target(host, port)

    private val commandLock = Any()
    private val channelLock = Any()

    private val commands =
        ArrayDeque<Packet>()

    private val channels =
        ArrayDeque<Packet>()

    private var commandFlight: Flight? = null
    private var channelFlight: Flight? = null
    private var latestNormal: Packet? = null
    private var channelBlocked = false
    private var lastReliableSequence = -1L
    private var lastPauseLevel: Int? = null

    @Volatile
    private var heartbeat: Heartbeat? = null

    private val heartbeatSequence =
        AtomicLong(0)

    private val superseded =
        AtomicLong(0)

    @Volatile
    var onReliableChannelRejected:
        ((sequence: Long, reason: String) -> Unit)? = null

    private val commandSocket =
        newSocket()

    private val channelSocket =
        newSocket()

    private val heartbeatSocket =
        newSocket()

    private val commandWorker =
        worker("snappy-udp-command")

    private val channelWorker =
        worker("snappy-udp-channel")

    private val heartbeatWorker =
        worker("snappy-udp-heartbeat")

    init {
        require(
            host.isNotBlank() &&
                port in 1..65535
        )

        schedule(
            commandWorker,
            TICK_MS
        ) {
            tickCommand()
        }

        schedule(
            channelWorker,
            TICK_MS
        ) {
            tickChannel()
        }

        schedule(
            heartbeatWorker,
            HEARTBEAT_MS
        ) {
            tickHeartbeat()
        }
    }

    fun normalChannelSupersededCount(): Long {
        return superseded.get()
    }

    fun targetText(): String {
        val current = target
        return "${current.host}:${current.port}"
    }

    fun updateTarget(
        host: String,
        port: Int
    ) {
        require(
            host.isNotBlank() &&
                port in 1..65535
        )

        if (closed.get()) {
            return
        }

        cancelReliableCommands()
        heartbeat = null
        target = Target(host, port)
    }

    fun send(
        command: CommandEnvelope,
        reliable: Boolean = false
    ): Boolean {
        if (closed.get()) {
            return false
        }

        if (
            command.action == "control.estop" ||
            command.action == "control.safe_hold" ||
            command.action == "control.disarm" ||
            command.action == "control.release"
        ) {
            cancelReliableCommands()
        }

        val payload =
            PacketCodec.encodeCommand(command)

        if (!fits(payload)) {
            return false
        }

        val packet =
            Packet(
                messageId = command.messageId,
                sequence = command.sequence,
                payload = payload,
                query = payload,
                queuedAt = now(),
                reliable = reliable
            )

        return synchronized(commandLock) {
            if (
                closed.get() ||
                commands.size >= MAX_QUEUE
            ) {
                false
            } else {
                commands.addLast(packet)
                true
            }
        }
    }

    fun acknowledgeCommand(
        messageId: String
    ) {
        if (
            messageId.isBlank() ||
            closed.get()
        ) {
            return
        }

        synchronized(commandLock) {
            if (
                commandFlight
                    ?.packet
                    ?.messageId == messageId
            ) {
                commandFlight = null
            }
        }
    }

    fun cancelReliableCommands() {
        synchronized(commandLock) {
            commands.clear()
            commandFlight = null
        }

        synchronized(channelLock) {
            channels.clear()
            channelFlight = null
            latestNormal = null
            channelBlocked = false
            lastReliableSequence = -1L
        }
    }

    fun sendChannels(
        robotId: String,
        sourceId: String,
        sessionId: String,
        sequence: Long,
        channels: List<Int>,
        deviceName: String,
        timestampMs: Long =
            System.currentTimeMillis(),
        reliable: Boolean = false
    ): Boolean {
        if (closed.get()) {
            return false
        }

        /*
         * timestampMs 来自 SDK 回调生成的快照。
         * 周期线程可能继续拿到旧快照，因此这里必须再次校验。
         * 使用 wall clock，因为 timestampMs 是 System.currentTimeMillis()。
         */
        val wallNow =
            System.currentTimeMillis()

        val inputAge =
            wallNow - timestampMs

        if (
            timestampMs <= 0L ||
            inputAge > INPUT_IDLE_MS ||
            inputAge < -MAX_FUTURE_SKEW_MS
        ) {
            if (reliable) {
                reject(
                    sequence,
                    "stale G20 input snapshot"
                )
            }

            Log.w(
                TAG,
                "drop stale G20 input " +
                    "seq=$sequence ageMs=$inputAge"
            )

            return false
        }

        val root =
            JSONObject()
                .put("schema_version", 1)
                .put("kind", "g20_channels")
                .put("robot_id", robotId)
                .put("source_id", sourceId)
                .put("session_id", sessionId)
                .put(
                    "message_id",
                    "g20-channels-" +
                        sessionId +
                        "-" +
                        sequence
                )
                .put("seq", sequence)
                .put("timestamp_ms", timestampMs)
                .put("device", deviceName)
                .put("channels", JSONArray(channels))

        if (reliable) {
            root.put("reliable", true)
        }

        val payload =
            root.toString()

        val query =
            JSONObject(payload)
                .put("delivery_query", true)
                .toString()

        if (
            !fits(payload) ||
            !fits(query)
        ) {
            if (reliable) {
                reject(
                    sequence,
                    "UDP payload exceeds 1200 bytes"
                )
            }

            return false
        }

        val time =
            now()

        val packet =
            Packet(
                messageId =
                    root.getString("message_id"),
                sequence = sequence,
                payload = payload,
                query = query,
                queuedAt = time,
                reliable = reliable
            )

        var failure: String? = null

        val accepted =
            synchronized(channelLock) {
                val pauseValue =
                    channels.getOrNull(
                        PAUSE_CHANNEL
                    ) ?: 1500

                val pauseLevel =
                    when {
                        pauseValue >=
                            HIGH_THRESHOLD -> 1

                        pauseValue <=
                            LOW_THRESHOLD -> -1

                        else -> 0
                    }

                val pauseEvent =
                    reliable &&
                        lastPauseLevel != null &&
                        pauseLevel != 0 &&
                        pauseLevel !=
                        lastPauseLevel

                if (reliable) {
                    lastPauseLevel =
                        pauseLevel
                }

                if (pauseEvent) {
                    this.channels.clear()
                    channelFlight = null
                    latestNormal = null
                    channelBlocked = false
                }

                when {
                    closed.get() -> {
                        false
                    }

                    !reliable -> {
                        if (latestNormal != null) {
                            superseded.incrementAndGet()
                        }

                        latestNormal = packet
                        true
                    }

                    channelBlocked -> {
                        failure =
                            "previous channel delivery is unresolved"
                        false
                    }

                    this.channels.size >= MAX_QUEUE -> {
                        channelBlocked = true
                        this.channels.clear()
                        latestNormal = null
                        failure =
                            "reliable channel queue full; " +
                                "recovery required"
                        false
                    }

                    else -> {
                        if (
                            latestNormal
                                ?.sequence
                                ?.let {
                                    it <= sequence
                                } == true
                        ) {
                            latestNormal = null
                            superseded.incrementAndGet()
                        }

                        this.channels.addLast(packet)
                        true
                    }
                }
            }

        /*
         * 只有成功进入发送队列的最新输入才刷新 keepalive。
         * 失败、过期或被阻塞的输入不能维持控制链路。
         */
        if (accepted) {
            heartbeat =
                Heartbeat(
                    robotId = robotId,
                    sourceId = sourceId,
                    sessionId = sessionId,
                    inputAt = time
                )
        }

        failure?.let {
            reject(sequence, it)
        }

        return accepted
    }

    fun acknowledgeChannel(
        messageId: String,
        sequence: Long,
        status: AckStatus,
        retryable: Boolean
    ) {
        var failure: String? = null

        synchronized(channelLock) {
            val flight =
                channelFlight ?: return

            if (
                flight.packet.messageId != messageId ||
                flight.packet.sequence != sequence
            ) {
                return
            }

            when (status) {
                AckStatus.ACCEPTED,
                AckStatus.SUCCEEDED -> {
                    channelFlight = null

                    if (flight.timedOut) {
                        channelBlocked = false
                    }
                }

                AckStatus.RUNNING -> {
                    flight.queryOnly = true
                    flight.nextAt =
                        minOf(
                            flight.nextAt,
                            now() + QUERY_MS
                        )
                }

                AckStatus.FAILED -> {
                    if (
                        retryable &&
                        !flight.timedOut
                    ) {
                        flight.queryOnly = false
                        flight.sends = 0
                        flight.nextAt =
                            now() + CHANNEL_RETRY_MS
                    } else {
                        channelFlight = null

                        if (flight.timedOut) {
                            channelBlocked = false
                        }

                        failure =
                            "gateway rejected channel event"
                    }
                }

                AckStatus.ABORTED -> {
                    channelFlight = null

                    if (flight.timedOut) {
                        channelBlocked = false
                    }

                    failure =
                        "channel event aborted"
                }

                AckStatus.TIMED_OUT -> {
                    flight.queryOnly = true

                    if (!flight.timedOut) {
                        flight.timedOut = true
                        channelBlocked = true
                        channels.clear()
                        latestNormal = null
                        failure =
                            "delivery outcome unknown; " +
                                "recovery required"
                    }
                }
            }
        }

        failure?.let {
            reject(sequence, it)
        }
    }

    private fun tickCommand() {
        val time =
            now()

        val packet =
            synchronized(commandLock) {
                val immediate =
                    commands.firstOrNull {
                        !it.reliable
                    }

                if (immediate != null) {
                    commands.remove(immediate)
                    return@synchronized immediate
                }

                val old =
                    commandFlight

                if (
                    old != null &&
                    time - old.startedAt >=
                    COMMAND_TIMEOUT_MS
                ) {
                    Log.w(
                        TAG,
                        "command ACK timeout: " +
                            old.packet.messageId
                    )

                    commandFlight = null
                }

                while (
                    commandFlight == null &&
                    commands.isNotEmpty()
                ) {
                    val next =
                        commands.removeFirst()

                    if (
                        time - next.queuedAt <=
                        COMMAND_TIMEOUT_MS
                    ) {
                        commandFlight =
                            Flight(next, time)
                    } else {
                        Log.w(
                            TAG,
                            "expired queued command: " +
                                next.messageId
                        )
                    }
                }

                val flight =
                    commandFlight

                if (
                    flight == null ||
                    time < flight.nextAt ||
                    flight.sends >= MAX_SENDS
                ) {
                    null
                } else {
                    flight.sends++
                    flight.nextAt =
                        time + COMMAND_RETRY_MS

                    if (!flight.packet.reliable) {
                        commandFlight = null
                    }

                    flight.packet
                }
            }

        if (packet != null) {
            sendPayload(
                commandSocket,
                packet.payload
            )
        }
    }

    private fun tickChannel() {
        val time =
            now()

        var failure:
            Pair<Long, String>? = null

        val payload =
            synchronized(channelLock) {
                if (
                    channelFlight == null &&
                    !channelBlocked &&
                    channels.isNotEmpty()
                ) {
                    val next =
                        channels.removeFirst()

                    if (
                        time - next.queuedAt >
                        CHANNEL_TIMEOUT_MS
                    ) {
                        channelBlocked = true
                        channels.clear()
                        latestNormal = null
                        failure =
                            next.sequence to
                                "queued channel event expired; " +
                                "recovery required"
                    } else {
                        channelFlight =
                            Flight(next, time)

                        lastReliableSequence =
                            next.sequence
                    }
                }

                val flight =
                    channelFlight

                if (flight != null) {
                    if (
                        !flight.timedOut &&
                        time - flight.startedAt >=
                        CHANNEL_TIMEOUT_MS
                    ) {
                        flight.timedOut = true
                        flight.queryOnly = true
                        channelBlocked = true
                        channels.clear()
                        latestNormal = null
                        failure =
                            flight.packet.sequence to
                                "channel ACK timeout; querying " +
                                "result, recovery may be required"
                    }

                    when {
                        time < flight.nextAt -> {
                            null
                        }

                        flight.queryOnly ||
                            flight.sends >= MAX_SENDS -> {
                            flight.queryOnly = true
                            flight.nextAt =
                                time + QUERY_MS
                            flight.packet.query
                        }

                        else -> {
                            flight.sends++
                            flight.nextAt =
                                time + CHANNEL_RETRY_MS
                            flight.packet.payload
                        }
                    }
                } else if (
                    !channelBlocked &&
                    channels.isEmpty()
                ) {
                    val normal =
                        latestNormal

                    latestNormal = null

                    if (
                        normal != null &&
                        normal.sequence >
                        lastReliableSequence &&
                        time - normal.queuedAt <=
                        INPUT_IDLE_MS
                    ) {
                        normal.payload
                    } else {
                        null
                    }
                } else {
                    null
                }
            }

        failure?.let {
            reject(it.first, it.second)
        }

        if (payload != null) {
            sendPayload(
                channelSocket,
                payload
            )
        }
    }

    private fun tickHeartbeat() {
        val current =
            heartbeat ?: return

        if (
            now() - current.inputAt >
            INPUT_IDLE_MS
        ) {
            return
        }

        val root =
            JSONObject()
                .put("schema_version", 1)
                .put("kind", "g20_keepalive")
                .put("robot_id", current.robotId)
                .put("source_id", current.sourceId)
                .put("session_id", current.sessionId)
                .put(
                    "seq",
                    heartbeatSequence.incrementAndGet()
                )

        sendPayload(
            heartbeatSocket,
            root.toString()
        )
    }

    private fun sendPayload(
        socket: DatagramSocket,
        payload: String
    ) {
        if (closed.get()) {
            return
        }

        try {
            val destination =
                target

            val bytes =
                payload.toByteArray(
                    Charsets.UTF_8
                )

            if (bytes.size > MAX_PAYLOAD) {
                return
            }

            val address =
                InetAddress.getByName(
                    destination.host
                )

            socket.send(
                DatagramPacket(
                    bytes,
                    bytes.size,
                    address,
                    destination.port
                )
            )
        } catch (error: Exception) {
            if (!closed.get()) {
                Log.w(
                    TAG,
                    "UDP send failed: " +
                        targetText(),
                    error
                )
            }
        }
    }

    private fun reject(
        sequence: Long,
        reason: String
    ) {
        Log.w(
            TAG,
            "channel rejected " +
                "seq=$sequence reason=$reason"
        )

        try {
            onReliableChannelRejected?.invoke(
                sequence,
                reason
            )
        } catch (error: Exception) {
            Log.e(
                TAG,
                "channel rejection callback failed",
                error
            )
        }
    }

    private fun schedule(
        executor: ScheduledExecutorService,
        periodMs: Long,
        task: () -> Unit
    ) {
        executor.scheduleWithFixedDelay(
            {
                if (!closed.get()) {
                    try {
                        task()
                    } catch (error: Exception) {
                        Log.e(
                            TAG,
                            "UDP worker failed",
                            error
                        )
                    }
                }
            },
            0L,
            periodMs,
            TimeUnit.MILLISECONDS
        )
    }

    fun close() {
        if (!closed.compareAndSet(false, true)) {
            return
        }

        commandSocket.close()
        channelSocket.close()
        heartbeatSocket.close()

        commandWorker.shutdownNow()
        channelWorker.shutdownNow()
        heartbeatWorker.shutdownNow()

        cancelReliableCommands()
        heartbeat = null
    }

    private fun fits(
        payload: String
    ): Boolean {
        return payload
            .toByteArray(Charsets.UTF_8)
            .size <= MAX_PAYLOAD
    }

    private fun now(): Long {
        return SystemClock.elapsedRealtime()
    }

    private fun newSocket(): DatagramSocket {
        return DatagramSocket().apply {
            broadcast = true
            sendBufferSize = 64 * 1024
        }
    }

    private fun worker(
        name: String
    ): ScheduledExecutorService {
        return Executors
            .newSingleThreadScheduledExecutor {
                runnable ->
                Thread(
                    runnable,
                    name
                ).apply {
                    isDaemon = true
                }
            }
    }

    companion object {
        private const val TAG =
            "SnappyUdpCommand"

        private const val PAUSE_CHANNEL = 12
        private const val LOW_THRESHOLD = 1300
        private const val HIGH_THRESHOLD = 1700

        private const val MAX_PAYLOAD = 1200
        private const val MAX_QUEUE = 32
        private const val MAX_SENDS = 4

        private const val TICK_MS = 10L
        private const val HEARTBEAT_MS = 100L

        private const val INPUT_IDLE_MS = 500L
        private const val MAX_FUTURE_SKEW_MS = 2000L

        private const val COMMAND_RETRY_MS = 250L
        private const val COMMAND_TIMEOUT_MS = 1000L

        private const val CHANNEL_RETRY_MS = 120L
        private const val QUERY_MS = 250L
        private const val CHANNEL_TIMEOUT_MS = 2000L
    }
}