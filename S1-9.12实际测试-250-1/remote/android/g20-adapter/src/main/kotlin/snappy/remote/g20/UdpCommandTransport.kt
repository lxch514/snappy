package snappy.remote.transport

import android.util.Log
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.ArrayDeque
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.RejectedExecutionException
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import org.json.JSONArray
import org.json.JSONObject
import snappy.protocol.AckStatus
import snappy.protocol.CommandEnvelope
import snappy.remote.g20.PacketCodec

class UdpCommandTransport(
    private var host: String,
    private var port: Int
) {
    private data class Target(
        val host: String,
        val port: Int
    )

    private data class ReliableCommandPacket(
        val messageId: String,
        val sequence: Long,
        val payload: String,
        val label: String
    )

    private data class ChannelPacket(
        val messageId: String,
        val sequence: Long,
        val payload: String,
        val label: String
    )

    private val closed = AtomicBoolean(false)
    private val targetLock = Any()

    private val socket =
        DatagramSocket().apply {
            broadcast = true
        }

    private val commandExecutor =
        Executors.newSingleThreadExecutor { runnable ->
            Thread(runnable, "snappy-udp-command").apply {
                isDaemon = true
            }
        }

    private val channelExecutor =
        Executors.newSingleThreadExecutor { runnable ->
            Thread(runnable, "snappy-udp-channel").apply {
                isDaemon = true
            }
        }

    private val retryExecutor =
        Executors.newSingleThreadScheduledExecutor { runnable ->
            Thread(runnable, "snappy-udp-retry").apply {
                isDaemon = true
            }
        }

    private val reliableCommandQueue =
        ArrayDeque<ReliableCommandPacket>()

    private var reliableCommandInFlight:
        ReliableCommandPacket? = null

    private val reliableChannelQueue =
        ArrayDeque<ChannelPacket>()

    private var reliableChannelInFlight:
        ChannelPacket? = null

    /*
     * RUNNING 表示 gateway/control 已经接受事件，
     * 但最终执行结果尚未确认。
     */
    private var reliableChannelAwaitingFinal:
        ChannelPacket? = null

    /*
     * 最终 ACK 超时后保留顺序屏障，
     * 不继续执行后续按键，避免未知结果造成重复动作。
     */
    private var reliableChannelFinalTimedOut = false

    /*
     * 普通周期舵量只保留最新值。
     */
    private var latestNormalChannel:
        ChannelPacket? = null

    @Volatile
    private var normalChannelSupersededCountValue: Long = 0

    /*
     * 队列满时通知上层。sendChannels() 返回值只表示任务是否提交到
     * channelExecutor，不表示可靠事件已经进入可靠队列。
     */
    @Volatile
    var onReliableChannelRejected:
        ((sequence: Long, reason: String) -> Unit)? = null

    fun normalChannelSupersededCount(): Long {
        return normalChannelSupersededCountValue
    }

    fun targetText(): String {
        val target = targetSnapshot()
        return "${target.host}:${target.port}"
    }

    fun updateTarget(
        host: String,
        port: Int
    ) {
        val oldTarget = targetText()

        synchronized(targetLock) {
            this.host = host
            this.port = port
        }

        Log.i(
            TAG,
            "updateTarget old=$oldTarget new=$host:$port"
        )
    }

    fun send(
        command: CommandEnvelope,
        reliable: Boolean = false
    ): Boolean {
        if (closed.get()) {
            Log.e(
                TAG,
                "send command rejected because transport is closed " +
                    "action=${command.action}"
            )
            return false
        }

        val payload = PacketCodec.encodeCommand(command)

        val label =
            "command action=${command.action} seq=${command.sequence}"

        if (!reliable) {
            return queueTask(
                executor = commandExecutor,
                label = label
            ) {
                sendPayload(
                    payload = payload,
                    label = label,
                    logSuccess = true
                )
            }
        }

        val packet =
            ReliableCommandPacket(
                messageId = command.messageId,
                sequence = command.sequence,
                payload = payload,
                label = label
            )

        return queueTask(
            executor = commandExecutor,
            label = label
        ) {
            reliableCommandQueue.addLast(packet)

            Log.i(
                TAG,
                "android_command_event=queued " +
                    "message=${packet.messageId} " +
                    "seq=${packet.sequence}"
            )

            sendNextReliableCommand()
        }
    }

    fun acknowledgeCommand(
        messageId: String
    ) {
        if (closed.get() || messageId.isBlank()) {
            return
        }

        queueTask(
            executor = commandExecutor,
            label = "command ack message=$messageId"
        ) {
            val current = reliableCommandInFlight

            if (
                current == null ||
                current.messageId != messageId
            ) {
                return@queueTask
            }

            reliableCommandInFlight = null

            Log.i(
                TAG,
                "android_command_event=ack " +
                    "message=$messageId " +
                    "seq=${current.sequence}"
            )

            sendNextReliableCommand()
        }
    }

    fun cancelReliableCommands() {
        if (closed.get()) {
            return
        }

        queueTask(
            executor = commandExecutor,
            label = "cancel reliable commands"
        ) {
            val current = reliableCommandInFlight

            reliableCommandQueue.clear()
            reliableCommandInFlight = null

            if (current != null) {
                Log.i(
                    TAG,
                    "android_command_event=cancelled " +
                        "message=${current.messageId}"
                )
            }
        }
    }

    private fun sendNextReliableCommand() {
        if (
            closed.get() ||
            reliableCommandInFlight != null ||
            reliableCommandQueue.isEmpty()
        ) {
            return
        }

        val packet = reliableCommandQueue.removeFirst()
        reliableCommandInFlight = packet

        sendPayload(
            payload = packet.payload,
            label = packet.label,
            logSuccess = false
        )

        scheduleCommandRetry(
            messageId = packet.messageId,
            retryAttempt = 1
        )
    }

    private fun scheduleCommandRetry(
        messageId: String,
        retryAttempt: Int
    ) {
        if (closed.get()) {
            return
        }

        try {
            retryExecutor.schedule(
                {
                    if (closed.get()) {
                        return@schedule
                    }

                    queueTask(
                        executor = commandExecutor,
                        label = "command retry message=$messageId"
                    ) {
                        handleCommandRetry(
                            messageId = messageId,
                            retryAttempt = retryAttempt
                        )
                    }
                },
                COMMAND_RETRY_DELAY_MS,
                TimeUnit.MILLISECONDS
            )
        } catch (error: RejectedExecutionException) {
            if (!closed.get()) {
                Log.e(
                    TAG,
                    "command retry schedule rejected " +
                        "message=$messageId",
                    error
                )
            }
        }
    }

    private fun handleCommandRetry(
        messageId: String,
        retryAttempt: Int
    ) {
        val current = reliableCommandInFlight

        if (
            current == null ||
            current.messageId != messageId
        ) {
            return
        }

        if (retryAttempt > COMMAND_MAX_RETRIES) {
            Log.e(
                TAG,
                "android_command_event=timeout " +
                    "message=$messageId " +
                    "retries=$COMMAND_MAX_RETRIES"
            )

            reliableCommandInFlight = null
            sendNextReliableCommand()
            return
        }

        sendPayload(
            payload = current.payload,
            label = "${current.label} retry=$retryAttempt",
            logSuccess = false
        )

        scheduleCommandRetry(
            messageId = messageId,
            retryAttempt = retryAttempt + 1
        )
    }

    fun sendChannels(
        robotId: String,
        sourceId: String,
        sessionId: String,
        sequence: Long,
        channels: List<Int>,
        deviceName: String,
        timestampMs: Long = System.currentTimeMillis(),
        reliable: Boolean = false
    ): Boolean {
        if (closed.get()) {
            Log.e(
                TAG,
                "send channels rejected because transport is closed " +
                    "seq=$sequence"
            )
            return false
        }

        val channelArray = JSONArray()
        channels.forEach { channelArray.put(it) }

        val messageId =
            "g20-channels-$sessionId-$sequence"

        val json =
            JSONObject()
                .put("schema_version", 1)
                .put("kind", "g20_channels")
                .put("robot_id", robotId)
                .put("source_id", sourceId)
                .put("session_id", sessionId)
                .put("message_id", messageId)
                .put("seq", sequence)
                .put("timestamp_ms", timestampMs)
                .put("device", deviceName)
                .put("channels", channelArray)

        if (reliable) {
            json.put("reliable", true)
        }

        val packet =
            ChannelPacket(
                messageId = messageId,
                sequence = sequence,
                payload = json.toString(),
                label = "g20_channels seq=$sequence"
            )

        return queueTask(
            executor = channelExecutor,
            label = packet.label
        ) {
            if (!reliable) {
                queueNormalChannel(packet)
                return@queueTask
            }

            if (
                reliableChannelQueue.size >=
                MAX_RELIABLE_CHANNEL_QUEUE
            ) {
                reportReliableChannelRejected(
                    sequence = sequence,
                    reason = "reliable queue full"
                )
                return@queueTask
            }

            reliableChannelQueue.addLast(packet)

            Log.i(
                TAG,
                "android_event=queued " +
                    "message=${packet.messageId} " +
                    "seq=${packet.sequence} " +
                    "channels=$channels"
            )

            sendNextReliableChannel()
        }
    }

    fun acknowledgeChannel(
        messageId: String,
        sequence: Long,
        status: AckStatus,
        retryable: Boolean
    ) {
        if (closed.get() || messageId.isBlank()) {
            return
        }

        queueTask(
            executor = channelExecutor,
            label =
                "g20_channels ack " +
                    "message=$messageId seq=$sequence"
        ) {
            val inFlight = reliableChannelInFlight
            val awaitingFinal = reliableChannelAwaitingFinal
            val current = inFlight ?: awaitingFinal

            if (
                current == null ||
                current.messageId != messageId ||
                current.sequence != sequence
            ) {
                Log.d(
                    TAG,
                    "android_event=ack_ignored " +
                        "message=$messageId " +
                        "seq=$sequence"
                )
                return@queueTask
            }

            when (status) {
                AckStatus.RUNNING -> {
                    if (inFlight != null) {
                        reliableChannelInFlight = null
                        reliableChannelAwaitingFinal = inFlight
                        reliableChannelFinalTimedOut = false

                        scheduleChannelFinalTimeout(
                            packet = inFlight
                        )
                    }
                }

                AckStatus.ACCEPTED -> {
                    /*
                     * 第一阶段同步受理。
                     * 等待最终结果时的迟到 ACCEPTED 必须忽略。
                     */
                    if (awaitingFinal == null) {
                        reliableChannelInFlight = null
                        reliableChannelFinalTimedOut = false
                        sendNextReliableChannel()
                    }
                }

                AckStatus.SUCCEEDED -> {
                    reliableChannelInFlight = null
                    reliableChannelAwaitingFinal = null
                    reliableChannelFinalTimedOut = false
                    sendNextReliableChannel()
                }

                AckStatus.FAILED -> {
                    if (retryable) {
                        if (awaitingFinal != null) {
                            reliableChannelAwaitingFinal = null
                            reliableChannelInFlight = awaitingFinal
                            reliableChannelFinalTimedOut = false
                            retryCurrentReliableChannelNow()
                        }
                    } else {
                        reliableChannelInFlight = null
                        reliableChannelAwaitingFinal = null
                        reliableChannelFinalTimedOut = false
                        sendNextReliableChannel()
                    }
                }

                AckStatus.ABORTED,
                AckStatus.TIMED_OUT -> {
                    if (retryable) {
                        if (awaitingFinal != null) {
                            reliableChannelAwaitingFinal = null
                            reliableChannelInFlight = awaitingFinal
                            reliableChannelFinalTimedOut = false
                            retryCurrentReliableChannelNow()
                        }
                    } else {
                        reliableChannelInFlight = null
                        reliableChannelAwaitingFinal = null
                        reliableChannelFinalTimedOut = false
                        sendNextReliableChannel()
                    }
                }
            }

            Log.i(
                TAG,
                "android_event=ack " +
                    "message=$messageId " +
                    "seq=$sequence " +
                    "status=$status " +
                    "retryable=$retryable"
            )
        }
    }

    private fun queueNormalChannel(
        packet: ChannelPacket
    ) {
        if (
            reliableChannelInFlight != null ||
            reliableChannelAwaitingFinal != null ||
            reliableChannelQueue.isNotEmpty() ||
            reliableChannelFinalTimedOut
        ) {
            latestNormalChannel = packet
            normalChannelSupersededCountValue += 1

            Log.d(
                TAG,
                "normal channel superseded " +
                    "seq=${packet.sequence} " +
                    "count=$normalChannelSupersededCountValue"
            )
            return
        }

        sendPayload(
            payload = packet.payload,
            label = packet.label,
            logSuccess = false
        )
    }

    private fun sendNextReliableChannel() {
        if (
            closed.get() ||
            reliableChannelInFlight != null ||
            reliableChannelAwaitingFinal != null ||
            reliableChannelFinalTimedOut
        ) {
            return
        }

        if (reliableChannelQueue.isNotEmpty()) {
            val packet = reliableChannelQueue.removeFirst()
            reliableChannelInFlight = packet

            sendPayload(
                payload = packet.payload,
                label = packet.label,
                logSuccess = false
            )

            scheduleChannelRetry(
                messageId = packet.messageId,
                sequence = packet.sequence,
                retryAttempt = 1
            )
            return
        }

        val latest = latestNormalChannel
        latestNormalChannel = null

        if (latest != null) {
            sendPayload(
                payload = latest.payload,
                label = latest.label,
                logSuccess = false
            )
        }
    }

    private fun scheduleChannelRetry(
        messageId: String,
        sequence: Long,
        retryAttempt: Int
    ) {
        if (closed.get()) {
            return
        }

        try {
            retryExecutor.schedule(
                {
                    if (closed.get()) {
                        return@schedule
                    }

                    queueTask(
                        executor = channelExecutor,
                        label =
                            "g20 retry " +
                                "message=$messageId seq=$sequence"
                    ) {
                        handleChannelRetry(
                            messageId = messageId,
                            sequence = sequence,
                            retryAttempt = retryAttempt
                        )
                    }
                },
                CHANNEL_RETRY_DELAY_MS,
                TimeUnit.MILLISECONDS
            )
        } catch (error: RejectedExecutionException) {
            if (!closed.get()) {
                Log.e(
                    TAG,
                    "channel retry schedule rejected " +
                        "message=$messageId",
                    error
                )
            }
        }
    }

    private fun handleChannelRetry(
        messageId: String,
        sequence: Long,
        retryAttempt: Int
    ) {
        val current = reliableChannelInFlight

        if (
            current == null ||
            current.messageId != messageId ||
           current.sequence != sequence
        ) {
            return
        }

        if (retryAttempt > CHANNEL_MAX_RETRIES) {
        /*
         * 不清除当前事件，也不发送后续事件。
         * 结果未知时保留顺序屏障，避免重复执行后续按键。
         * 迟到的 SUCCEEDED/FAILED ACK 仍可通过
         * reliableChannelAwaitingFinal 匹配。
         */
            reliableChannelInFlight = null
            reliableChannelAwaitingFinal = current
            reliableChannelFinalTimedOut = true

            reportReliableChannelRejected(
                sequence = current.sequence,
                reason = "delivery result unknown after retry timeout"
            )

            Log.e(
                TAG,
                "android_event=timeout " +
                    "message=$messageId " +
                    "seq=$sequence " +
                    "retries=$CHANNEL_MAX_RETRIES"
            )

            return
        }

        sendPayload(
            payload = current.payload,
            label = "${current.label} retry=$retryAttempt",
            logSuccess = false
        )

        scheduleChannelRetry(
            messageId = messageId,
            sequence = sequence,
            retryAttempt = retryAttempt + 1
        )
    }

    private fun retryCurrentReliableChannelNow() {
        val current = reliableChannelInFlight ?: return

        reliableChannelFinalTimedOut = false

        sendPayload(
            payload = current.payload,
            label = "${current.label} immediate-retry",
            logSuccess = false
        )

        scheduleChannelRetry(
            messageId = current.messageId,
            sequence = current.sequence,
            retryAttempt = 1
        )
    }

    private fun scheduleChannelFinalTimeout(
        packet: ChannelPacket
    ) {
        try {
            retryExecutor.schedule(
                {
                    queueTask(
                        executor = channelExecutor,
                        label =
                            "g20 final timeout " +
                                "message=${packet.messageId}"
                    ) {
                        val current =
                            reliableChannelAwaitingFinal

                        if (
                            current != null &&
                            current.messageId == packet.messageId &&
                            current.sequence == packet.sequence
                        ) {
                            reliableChannelFinalTimedOut = true

                            reportReliableChannelRejected(
                                sequence = packet.sequence,
                                reason = "downstream result unknown"
                            )

                            Log.e(
                                TAG,
                                "android_event=final_timeout " +
                                    "message=${packet.messageId} " +
                                    "seq=${packet.sequence}"
                            )
                        }
                    }
                },
                CHANNEL_FINAL_TIMEOUT_MS,
                TimeUnit.MILLISECONDS
            )
        } catch (error: RejectedExecutionException) {
            if (!closed.get()) {
                Log.e(
                    TAG,
                    "final ACK timeout schedule rejected " +
                        "message=${packet.messageId}",
                    error
                )
            }
        }
    }

    private fun reportReliableChannelRejected(
        sequence: Long,
        reason: String
    ) {
        Log.e(
            TAG,
            "reliable channel event rejected " +
                "seq=$sequence reason=$reason"
        )

        try {
            onReliableChannelRejected?.invoke(
                sequence,
                reason
            )
        } catch (error: Throwable) {
            Log.e(
                TAG,
                "reliable rejection callback failed",
                error
            )
        }
    }

    private fun sendPayload(
        payload: String,
        label: String,
        logSuccess: Boolean
    ): Boolean {
        if (closed.get()) {
            return false
        }

        return try {
            val target = targetSnapshot()
            val address = InetAddress.getByName(target.host)
            val bytes = payload.toByteArray(Charsets.UTF_8)

            val packet =
                DatagramPacket(
                    bytes,
                    bytes.size,
                    address,
                    target.port
                )

            synchronized(socket) {
                if (closed.get()) {
                    return false
                }

                socket.send(packet)
            }

            if (logSuccess) {
                Log.i(
                    TAG,
                    "send ok label=$label " +
                        "target=${address.hostAddress}:${target.port} " +
                        "bytes=${bytes.size}"
                )
            }

            true
        } catch (error: Throwable) {
            if (!closed.get()) {
                Log.e(
                    TAG,
                    "send failed label=$label " +
                        "target=${targetText()} " +
                        "payload=${payload.take(240)}",
                    error
                )
            }

            false
        }
    }

    private fun queueTask(
        executor: ExecutorService,
        label: String,
        task: () -> Unit
    ): Boolean {
        if (closed.get()) {
            return false
        }

        return try {
            executor.execute {
                if (!closed.get()) {
                    task()
                }
            }
            true
        } catch (error: RejectedExecutionException) {
            if (!closed.get()) {
                Log.e(
                    TAG,
                    "queue rejected label=$label",
                    error
                )
            }
            false
        } catch (error: Throwable) {
            Log.e(
                TAG,
                "queue failed label=$label",
                error
            )
            false
        }
    }

    private fun targetSnapshot(): Target {
        return synchronized(targetLock) {
            Target(host, port)
        }
    }

    fun close() {
        if (!closed.compareAndSet(false, true)) {
            return
        }

        retryExecutor.shutdownNow()
        channelExecutor.shutdownNow()
        commandExecutor.shutdownNow()

        try {
            socket.close()
        } catch (_: Throwable) {
        }
    }

    companion object {
        private const val TAG = "SnappyUdpCommand"

        private const val COMMAND_RETRY_DELAY_MS = 250L
        private const val COMMAND_MAX_RETRIES = 3

        private const val CHANNEL_RETRY_DELAY_MS = 120L
        private const val CHANNEL_MAX_RETRIES = 3

        private const val CHANNEL_FINAL_TIMEOUT_MS = 2000L
        private const val MAX_RELIABLE_CHANNEL_QUEUE = 32
    }
}