/**
 * @file RemoteViewModel.kt
 * @brief 遥控界面状态管理
 *
 * @details
 * - 保持原有按键、摇杆、界面和执行机构控制逻辑。
 * - G20 动作舵量以及控制权恢复命令使用带确认的可靠发送。
 * - 状态与遥测只读取接收线程解出的信封，不直接碰网络。
 */

package snappy.remote.app

import android.util.Log
import java.util.UUID
import java.util.concurrent.CopyOnWriteArrayList
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong
import snappy.protocol.AckEnvelope
import snappy.protocol.AckStatus
import snappy.protocol.CommandEnvelope
import snappy.protocol.RemoteAction
import snappy.protocol.StatusEnvelope
import snappy.protocol.TelemetryEnvelope
import snappy.protocol.VideoStateDto
import snappy.remote.g20.G20Adapter
import snappy.remote.g20.G20ChannelSnapshot
import snappy.remote.g20.G20RemoteDefaults
import snappy.remote.g20.NullG20Adapter
import snappy.remote.g20.UdpStatusReceiver
import snappy.remote.g20.UdpTelemetryReceiver
import snappy.remote.transport.UdpCommandTransport

class RemoteViewModel(
    private val robotId: String = G20RemoteDefaults.ROBOT_ID,
    private val g20Adapter: G20Adapter = NullG20Adapter(),
    private val commandTransport: UdpCommandTransport =
        UdpCommandTransport(
            G20RemoteDefaults.UDP_COMMAND_HOST,
            G20RemoteDefaults.UDP_COMMAND_PORT
        ),
    private val statusReceiver: UdpStatusReceiver =
        UdpStatusReceiver(),
    private val telemetryReceiver: UdpTelemetryReceiver =
        UdpTelemetryReceiver()
) {
    /*
     * 一次屏幕控制权恢复包含 acquire 和 arm 两个阶段。
     * 使用准确 message_id 匹配 ACK，避免旧 ACK 干扰新流程。
     */
    private data class RecoveryAttempt(
        val acquireMessageId: String,
        val acquireSequence: Long,
        val armMessageId: String? = null,
        val armSequence: Long? = null
    )

    private val running = AtomicBoolean(false)
    private val sequence = AtomicLong(1)
    private val sessionId = UUID.randomUUID().toString()
    private val sourceId = G20Adapter.DEFAULT_SOURCE_ID
    private val observers =
        CopyOnWriteArrayList<(RemoteUiState) -> Unit>()

    private val snapshotLock = Object()
    private val stateLock = Object()
    private val leaseLock = Any()
    private val recoveryLock = Any()
    private val telemetryPacketCount = AtomicLong(0)

    @Volatile
    private var lastSnapshot: G20ChannelSnapshot? = null

    /*
     * 摇杆采样会在几毫秒内吐出跳变值，让同一次操作产生
     * “进舵区→落死区→再进舵区”的假边沿。
     * 记录每个动作通道上次舵位变化的时间，窗口内的来回翻转按噪声丢弃。
     */
    private val stableLevels =
        IntArray(G20Adapter.CHANNEL_COUNT)

    private val stableChangedAt =
        LongArray(G20Adapter.CHANNEL_COUNT)

    @Volatile
    private var leaseId: String? = null

    @Volatile
    private var controlEpoch: Long = 0

    private var recoveryAttempt: RecoveryAttempt? = null
    private var channelPumpThread: Thread? = null

    var onVideoState: ((VideoStateDto) -> Unit)? = null
    var onG20Channels: ((G20ChannelSnapshot) -> Unit)? = null

    @Volatile
    var uiState: RemoteUiState = RemoteUiState()
        private set

    fun observe(observer: (RemoteUiState) -> Unit) {
        observers += observer
        observer(currentState())
    }

    fun start(): Boolean {
        if (!running.compareAndSet(false, true)) {
            Log.i(TAG, "start ignored, already running")
            return true
        }

        Log.i(
            TAG,
            "start robotId=$robotId sourceId=$sourceId " +
                "sessionId=$sessionId " +
                "target=${commandTransport.targetText()}"
        )

        updateState { state ->
            state.copy(
                rcStatus = "G20: connecting",
                linkStatus = "link: starting",
                commandStatus = "command: waiting",
                telemetryStatus =
                    "telemetry: waiting for 14552"
            )
        }

        startStatusReceiver()
        startTelemetryReceiver()

        val adapterStarted =
            g20Adapter.start { snapshot ->
                onG20Snapshot(snapshot)
            }

        Log.i(
            TAG,
            "g20Adapter.start result=$adapterStarted " +
                "status=${g20Adapter.statusText}"
        )

        startChannelPump()

        updateState { state ->
            state.copy(
                rcStatus = g20Adapter.statusText,
                linkStatus = if (adapterStarted) {
                    "link: UDP sending"
                } else {
                    "link: G20 unavailable"
                },
                commandStatus = if (adapterStarted) {
                    "command: G20 connected"
                } else {
                    "command: G20 unavailable"
                }
            )
        }

        return adapterStarted
    }

    fun stop() {
        Log.i(TAG, "stop requested")

        running.set(false)
        channelPumpThread?.interrupt()
        channelPumpThread = null

        cancelRecovery(
            reason = "view model stopped",
            cancelTransport = true
        )

        clearLease()

        statusReceiver.stop()
        telemetryReceiver.stop()
        g20Adapter.stop()
        commandTransport.close()

        updateState { state ->
            state.copy(
                rcStatus = "G20: stopped",
                linkStatus = "link: stopped",
                telemetryStatus = "telemetry: stopped",
                videoPlayerStatus = "player: stopped"
            )
        }
    }

    /* ============================================================
     * 控制租约
     * ============================================================ */

    private fun leaseSnapshot(): Pair<String?, Long> {
        return synchronized(leaseLock) {
            leaseId to controlEpoch
        }
    }

    private fun updateLease(
        id: String,
        epoch: Long
    ) {
        synchronized(leaseLock) {
            leaseId = id
            controlEpoch = epoch
        }
    }

    private fun clearLease() {
        synchronized(leaseLock) {
            leaseId = null
            controlEpoch = 0
        }
    }

    /* ============================================================
     * 屏幕控制权恢复
     * ============================================================ */

    /*
     * 一次点击开始新的 acquire -> arm 流程。
     * 旧的可靠恢复命令和旧流程状态先被取消。
     */
    fun acquireAndArm(): Boolean {
        commandTransport.cancelReliableCommands()

        val acquireSequence =
            sequence.getAndIncrement()

        val acquireMessageId =
            "${RemoteAction.ControlAcquire.wireValue}-" +
                acquireSequence

        synchronized(recoveryLock) {
            recoveryAttempt = RecoveryAttempt(
                acquireMessageId = acquireMessageId,
                acquireSequence = acquireSequence
            )
        }

        Log.i(
            TAG,
            "recovery begin " +
                "acquire_message=$acquireMessageId"
        )

        val sent = sendPreparedAction(
            action = RemoteAction.ControlAcquire,
            params = emptyMap(),
            sequenceValue = acquireSequence,
            reliable = true
        )

        if (!sent) {
            clearRecoveryIfMatches(
                acquireMessageId
            )
        }

        return sent
    }

    private fun sendArmForRecovery(
        acquireMessageId: String
    ): Boolean {
        val armSequence =
            sequence.getAndIncrement()

        val armMessageId =
            "${RemoteAction.ControlArm.wireValue}-$armSequence"

        val accepted =
            synchronized(recoveryLock) {
                val current = recoveryAttempt

                if (
                    current == null ||
                    current.acquireMessageId !=
                    acquireMessageId
                ) {
                    false
                } else {
                    recoveryAttempt = current.copy(
                        armMessageId = armMessageId,
                        armSequence = armSequence
                    )
                    true
                }
            }

        if (!accepted) {
            Log.w(
                TAG,
                "recovery arm skipped because " +
                    "acquire is no longer current " +
                    "message=$acquireMessageId"
            )
            return false
        }

        Log.i(
            TAG,
            "recovery acquire acknowledged; " +
                "send arm message=$armMessageId"
        )

        val sent = sendPreparedAction(
            action = RemoteAction.ControlArm,
            params = emptyMap(),
            sequenceValue = armSequence,
            reliable = true
        )

        if (!sent) {
            clearRecoveryIfMatches(
                armMessageId
            )
        }

        return sent
    }

    private fun recoverySnapshot(): RecoveryAttempt? {
        return synchronized(recoveryLock) {
            recoveryAttempt
        }
    }

    private fun clearRecoveryIfMatches(
        messageId: String
    ): Boolean {
        return synchronized(recoveryLock) {
            val current = recoveryAttempt

            val matches =
                current?.acquireMessageId == messageId ||
                    current?.armMessageId == messageId

            if (matches) {
                recoveryAttempt = null
            }

            matches
        }
    }

    private fun cancelRecovery(
        reason: String,
        cancelTransport: Boolean
    ) {
        val cancelled =
            synchronized(recoveryLock) {
                val current = recoveryAttempt
                recoveryAttempt = null
                current
            }

        if (cancelTransport) {
            commandTransport.cancelReliableCommands()
        }

        if (cancelled != null) {
            Log.i(
                TAG,
                "recovery cancelled reason=$reason " +
                    "acquire=${cancelled.acquireMessageId} " +
                    "arm=${cancelled.armMessageId ?: "-"}"
            )
        }
    }

    /* ============================================================
     * 屏幕普通动作发送
     * ============================================================ */

    fun sendAction(
        action: RemoteAction,
        params: Map<String, String>
    ): Boolean {
        /*
         * 用户在屏幕上主动执行安全停止、释放或清故障时，
         * 必须取消尚未完成的 acquire/arm 重试。
         */
        if (actionCancelsRecovery(action)) {
            cancelRecovery(
                reason =
                    "local action ${action.wireValue}",
                cancelTransport = true
            )
        }

        return sendPreparedAction(
            action = action,
            params = params,
            sequenceValue =
                sequence.getAndIncrement(),
            reliable = false
        )
    }

    private fun actionCancelsRecovery(
        action: RemoteAction
    ): Boolean {
        return action == RemoteAction.ControlSafeHold ||
            action == RemoteAction.ControlEstop ||
            action == RemoteAction.ControlRelease ||
            action == RemoteAction.FaultClear
    }

    private fun sendPreparedAction(
        action: RemoteAction,
        params: Map<String, String>,
        sequenceValue: Long,
        reliable: Boolean
    ): Boolean {
        val currentLease =
            leaseSnapshot()

        val needsLease =
            action != RemoteAction.ControlAcquire &&
                !(
                    action == RemoteAction.FaultClear &&
                        currentLease.first.isNullOrBlank()
                    )

        val messageId =
            "${action.wireValue}-$sequenceValue"

        val command = CommandEnvelope.of(
            robotId = robotId,
            sourceId = sourceId,
            sessionId = sessionId,
            messageId = messageId,
            sequence = sequenceValue,
            action = action,
            params = params,
            leaseId =
                if (needsLease) {
                    currentLease.first
                } else {
                    null
                },
            controlEpoch =
                if (needsLease) {
                    currentLease.second
                } else {
                    0
                }
        )

        Log.i(
            TAG,
            "sendAction action=${action.wireValue} " +
                "message=$messageId " +
                "seq=$sequenceValue " +
                "reliable=$reliable " +
                "needsLease=$needsLease " +
                "lease=${currentLease.first ?: "-"} " +
                "epoch=${currentLease.second} " +
                "target=${commandTransport.targetText()} " +
                "params=$params"
        )

        val sent =
            commandTransport.send(
                command = command,
                reliable = reliable
            )

        updateState { state ->
            state.copy(
                commandStatus = if (sent) {
                    "command: sent ${action.wireValue}"
                } else {
                    "command: failed ${action.wireValue}"
                }
            )
        }

        return sent
    }

    fun updateTarget(
        host: String,
        port: Int
    ) {
        Log.i(TAG, "updateTarget $host:$port")

        commandTransport.updateTarget(
            host,
            port
        )

        updateState { state ->
            state.copy(
                linkStatus = "link: $host:$port"
            )
        }
    }

    fun setVideoStatus(status: String) {
        updateState { state ->
            state.copy(videoStatus = status)
        }
    }

    fun setVideoPlayerStatus(status: String) {
        updateState { state ->
            state.copy(videoPlayerStatus = status)
        }
    }

    fun setVideoUrl(url: String) {
        updateState { state ->
            state.copy(videoUrl = url)
        }
    }

    /* ============================================================
     * ACK 和状态接收
     * ============================================================ */

    private fun startStatusReceiver() {
        statusReceiver.start(
            object : UdpStatusReceiver.Listener {
                override fun onAck(ack: AckEnvelope) {
                    /*
                     * 舵量事件 ACK 只用于结束舵量重试，
                     * 不参与控制租约和界面命令状态处理。
                     */
                    if (
                        ack.messageId.startsWith(
                            G20_CHANNEL_MESSAGE_PREFIX
                        )
                    ) {
                        commandTransport
                            .acknowledgeChannel(
                                messageId = ack.messageId,
                                sequence = ack.sequence,
                                status = ack.status,
                                retryable = ack.retryable
                            )

                        Log.i(
                            TAG,
                            "g20 event ack received " +
                                "message=${ack.messageId} " +
                                "seq=${ack.sequence} " +
                                "status=${ack.status} " +
                                "retryable=${ack.retryable} " +
                                "detail=${ack.detail}"
                        )

                        return
                    }

                    /*
                     * 可靠 acquire/arm 收到任意终态 ACK 后，
                     * UDP 传输层停止重试。
                     */
                    commandTransport
                        .acknowledgeCommand(
                            ack.messageId
                        )

                    val recovery =
                        recoverySnapshot()

                    val isAnyAcquireAck =
                        ack.messageId.startsWith(
                            RemoteAction
                                .ControlAcquire
                                .wireValue
                        )

                    val isCurrentAcquireAck =
                        recovery?.acquireMessageId ==
                            ack.messageId

                    val isCurrentArmAck =
                        recovery?.armMessageId ==
                            ack.messageId

                    /*
                     * 旧 acquire ACK 不能覆盖当前恢复流程的租约。
                     * 没有恢复流程时，允许接收 mapper 初始化租约。
                     */
                    val mayUpdateLease =
                        !isAnyAcquireAck ||
                            isCurrentAcquireAck ||
                            recovery == null

                    if (
                        mayUpdateLease &&
                        ack.leaseId.isNotBlank()
                    ) {
                        updateLease(
                            id = ack.leaseId,
                            epoch = ack.controlEpoch
                        )
                    }

                    Log.i(
                        TAG,
                        "ack received " +
                            "status=${ack.status} " +
                            "message=${ack.messageId} " +
                            "lease=${ack.leaseId} " +
                            "epoch=${ack.controlEpoch} " +
                            "currentAcquire=" +
                            isCurrentAcquireAck +
                            " currentArm=" +
                            isCurrentArmAck +
                            " detail=${ack.detail}"
                    )

                    var shouldSendArm = false
                    var shouldReacquireAfterFaultClear =
                        false

                    /*
                     * 只有当前 acquire 的准确 message_id
                     * 才能触发自动 arm。
                     */
                    if (isCurrentAcquireAck) {
                        when (ack.status) {
                            AckStatus.SUCCEEDED -> {
                                val lease =
                                    leaseSnapshot()

                                if (
                                    !lease.first
                                        .isNullOrBlank()
                                ) {
                                    shouldSendArm = true
                                } else {
                                    Log.e(
                                        TAG,
                                        "recovery acquire " +
                                            "succeeded without lease"
                                    )

                                    clearRecoveryIfMatches(
                                        ack.messageId
                                    )
                                }
                            }

                            AckStatus.FAILED,
                            AckStatus.ABORTED,
                            AckStatus.TIMED_OUT -> {
                                Log.e(
                                    TAG,
                                    "recovery acquire failed " +
                                        "status=${ack.status} " +
                                        "detail=${ack.detail}"
                                )

                                clearRecoveryIfMatches(
                                    ack.messageId
                                )
                                clearLease()
                            }

                            AckStatus.ACCEPTED,
                            AckStatus.RUNNING -> {
                                /*
                                 * 中间状态继续等待最终 ACK。
                                 */
                            }
                        }
                    }

                    /*
                     * arm 成功或失败都结束本次恢复流程。
                     */
                    if (isCurrentArmAck) {
                        when (ack.status) {
                            AckStatus.SUCCEEDED -> {
                                clearRecoveryIfMatches(
                                    ack.messageId
                                )

                                Log.i(
                                    TAG,
                                    "recovery completed " +
                                        "arm=${ack.messageId}"
                                )
                            }

                            AckStatus.FAILED,
                            AckStatus.ABORTED,
                            AckStatus.TIMED_OUT -> {
                                clearRecoveryIfMatches(
                                    ack.messageId
                                )

                                Log.e(
                                    TAG,
                                    "recovery arm failed " +
                                        "status=${ack.status} " +
                                        "detail=${ack.detail}"
                                )
                            }

                            AckStatus.ACCEPTED,
                            AckStatus.RUNNING -> {
                                /*
                                 * 中间状态继续等待最终 ACK。
                                 */
                            }
                        }
                    }

                    val isSafeHoldAck =
                        ack.messageId.startsWith(
                            RemoteAction
                                .ControlSafeHold
                                .wireValue
                        )

                    val isEstopAck =
                        ack.messageId.startsWith(
                            RemoteAction
                                .ControlEstop
                                .wireValue
                        )

                    val isReleaseAck =
                        ack.messageId.startsWith(
                            RemoteAction
                                .ControlRelease
                                .wireValue
                        )

                    val isFaultClearAck =
                        ack.messageId.startsWith(
                            RemoteAction
                                .FaultClear
                                .wireValue
                        )

                    if (
                        ack.status ==
                        AckStatus.SUCCEEDED
                    ) {
                        if (
                            isSafeHoldAck ||
                            isEstopAck ||
                            isReleaseAck
                        ) {
                            /*
                             * 遥控暂停 ACK 可能在用户已经点击控制权后
                             * 才到达。只要新的恢复流程正在进行，
                             * 旧暂停 ACK 就不能将其取消。
                             *
                             * 屏幕主动安全停止会在发送前调用
                             * cancelRecovery，因此仍会正常清除租约。
                             */
                            if (
                                recoverySnapshot() == null
                            ) {
                                clearLease()
                            } else {
                                Log.i(
                                    TAG,
                                    "delayed safety ACK " +
                                        "did not cancel newer " +
                                        "recovery message=" +
                                        ack.messageId
                                )
                            }
                        }

                        if (isFaultClearAck) {
                            clearLease()
                            shouldReacquireAfterFaultClear =
                                true
                        }
                    }

                    updateState { state ->
                        state.copy(
                            commandStatus =
                                "ACK: ${ack.status} " +
                                    ack.detail
                        )
                    }

                    if (shouldSendArm) {
                        val armSent =
                            sendArmForRecovery(
                                ack.messageId
                            )

                        if (!armSent) {
                            Log.e(
                                TAG,
                                "automatic control.arm " +
                                    "send failed"
                            )
                        }
                    } else if (
                        shouldReacquireAfterFaultClear
                    ) {
                        val acquireSent =
                            acquireAndArm()

                        if (!acquireSent) {
                            Log.e(
                                TAG,
                                "automatic control.acquire " +
                                    "after fault.clear failed"
                            )
                        }
                    }
                }

                override fun onStatus(
                    status: StatusEnvelope
                ) {
                    if (
                        status.leaseHeld &&
                        status.leaseId.isNotBlank()
                    ) {
                        synchronized(leaseLock) {
                            if (
                                leaseId.isNullOrBlank() &&
                                (
                                    status.leaseOwner
                                        .isBlank() ||
                                        status.leaseOwner ==
                                        sourceId
                                    )
                            ) {
                                leaseId =
                                    status.leaseId
                                controlEpoch =
                                    status.controlEpoch
                            }
                        }
                    }

                    updateState { state ->
                        state.copy(
                            status = status,
                            actuators =
                                status.actuators,
                            deviceStatus =
                                formatDeviceStatus(
                                    status
                                ),
                            linkStatus =
                                if (status.g20Online) {
                                    "link: online"
                                } else {
                                    "link: offline"
                                },
                            safetyStatus =
                                formatSafetyStatus(
                                    status
                                ),
                            watchdogStatus =
                                formatWatchdogStatus(
                                    status
                                ),
                            protocolStatus =
                                formatProtocolStatus(
                                    status
                                )
                        )
                    }
                }

                override fun onVideo(
                    video: VideoStateDto
                ) {
                    val nextUrl =
                        videoUrlFor(video)

                    updateState { state ->
                        state.copy(
                            videoStatus =
                                formatVideoStatus(
                                    video
                                ),
                            videoUrl =
                                nextUrl.ifBlank {
                                    state.videoUrl
                                }
                        )
                    }

                    onVideoState?.invoke(video)
                }

                override fun onRawStatus(
                    payload: String
                ) {
                    Log.i(
                        TAG,
                        "status raw payload=" +
                            payload.take(240)
                    )

                    updateState { state ->
                        state.copy(
                            deviceStatus =
                                "status raw: " +
                                    payload.take(160)
                        )
                    }
                }

                override fun onStatusError(
                    error: String
                ) {
                    Log.e(
                        TAG,
                        "status receive error: $error"
                    )

                    updateState { state ->
                        state.copy(
                            deviceStatus =
                                "status receive error: " +
                                    error
                        )
                    }
                }
            }
        )
    }

    /* ============================================================
     * 遥测接收
     * ============================================================ */

    private fun startTelemetryReceiver() {
        val started =
            telemetryReceiver.start(
                object :
                    UdpTelemetryReceiver.Listener {
                    override fun onTelemetry(
                        telemetry:
                            TelemetryEnvelope
                    ) {
                        val count =
                            telemetryPacketCount
                                .incrementAndGet()

                        Log.i(
                            TAG,
                            "telemetry received " +
                                "count=$count " +
                                "samples=" +
                                telemetry.samples.size
                        )

                        updateState { state ->
                            state.copy(
                                telemetry =
                                    telemetry,
                                telemetryStatus =
                                    "telemetry: received " +
                                        "$count packet(s)",
                                deviceStatus =
                                    "device: telemetry " +
                                        "updated"
                            )
                        }
                    }

                    override fun onRawTelemetry(
                        payload: String
                    ) {
                        val count =
                            telemetryPacketCount
                                .incrementAndGet()

                        Log.i(
                            TAG,
                            "telemetry raw " +
                                "count=$count " +
                                "payload=" +
                                payload.take(200)
                        )

                        updateState { state ->
                            state.copy(
                                telemetryStatus =
                                    "telemetry: raw " +
                                        "packet $count",
                                deviceStatus =
                                    "device: telemetry " +
                                        "raw packet not parsed"
                            )
                        }
                    }

                    override fun onTelemetryError(
                        error: String
                    ) {
                        Log.e(
                            TAG,
                            "telemetry error: $error"
                        )

                        updateState { state ->
                            state.copy(
                                telemetryStatus =
                                    "telemetry: error",
                                deviceStatus =
                                    "telemetry receive " +
                                        "error: $error"
                            )
                        }
                    }
                }
            )

        Log.i(
            TAG,
            "telemetryReceiver.start " +
                "result=$started"
        )

        if (!started) {
            updateState { state ->
                state.copy(
                    telemetryStatus =
                        "telemetry: listener " +
                            "start failed"
                )
            }
        }
    }

    /* ============================================================
     * G20 舵量发送
     * ============================================================ */

    private fun onG20Snapshot(
        snapshot: G20ChannelSnapshot
    ) {
        val normalized =
            snapshot.copy(
                values =
                    snapshot.normalizedChannels()
            )

        val previous =
            synchronized(snapshotLock) {
                val value = lastSnapshot
                lastSnapshot = normalized
                value
            }

        val reliable =
            previous == null ||
                stableActionChanged(
                    normalized.normalizedChannels()
                )

        /*
         * 动作通道变化立即发送。
         * 普通模拟量由 100ms 周期线程发送，避免重复占满 UDP。
         */
        if (reliable) {
            Log.i(
                TAG,
                "g20 action-channel event " +
                    "previous=" +
                    (
                        previous
                            ?.normalizedChannels()
                            ?: emptyList<Int>()
                        ) +
                    " current=" +
                    normalized.normalizedChannels()
            )

            sendSnapshot(
                snapshot = normalized,
                reliable = true
            )
        }

        onG20Channels?.invoke(normalized)

        updateState { state ->
            state.copy(
                rcStatus =
                    g20Adapter.statusText,
                channelStatus =
                    formatChannels(
                        normalized
                            .normalizedChannels()
                    )
            )
        }
    }

    private fun startChannelPump() {
        channelPumpThread = Thread {
            while (running.get()) {
                val snapshot =
                    synchronized(snapshotLock) {
                        lastSnapshot
                    }

                if (snapshot != null) {
                    sendSnapshot(
                        snapshot = snapshot,
                        reliable = false
                    )

                    onG20Channels?.invoke(
                        snapshot
                    )
                }

                try {
                    Thread.sleep(
                        CHANNEL_PUMP_INTERVAL_MS
                    )
                } catch (_: InterruptedException) {
                    return@Thread
                }
            }
        }.apply {
            name =
                "snappy-g20-channel-pump"
            isDaemon = true
            start()
        }
    }

    private fun sendSnapshot(
        snapshot: G20ChannelSnapshot,
        reliable: Boolean
    ): Boolean {
        val sent =
            commandTransport.sendChannels(
                robotId = robotId,
                sourceId = sourceId,
                sessionId = sessionId,
                sequence =
                    sequence.getAndIncrement(),
                channels =
                    snapshot.normalizedChannels(),
                deviceName =
                    snapshot.deviceName,
                timestampMs =
                    snapshot.timestampMs,
                reliable = reliable
            )

        if (!sent) {
            Log.e(
                TAG,
                "sendSnapshot failed " +
                    "reliable=$reliable " +
                    "target=" +
                    commandTransport.targetText()
            )

            updateState { state ->
                state.copy(
                    commandStatus =
                        "command: G20 channel " +
                            "send failed"
                )
            }
        }

        return sent
    }

    private fun reliableInputChanged(
        previous: List<Int>,
        current: List<Int>
    ): Boolean {
        return ACTION_CHANNELS.any { index ->
            inputLevel(
                previous.getOrElse(index) {
                    CHANNEL_NEUTRAL
                }
            ) != inputLevel(
                current.getOrElse(index) {
                    CHANNEL_NEUTRAL
                }
            )
        }
    }

    /*
     * 用去抖后的动作通道舵位判断是否需要立即发送可靠帧。
     * 距上次变化不足 STICK_DEBOUNCE_MS 的翻转按噪声丢弃，
     * 正常推动只会在稳定后触发一次。
     */
    private fun stableActionChanged(
        current: List<Int>
    ): Boolean {
        val now = System.currentTimeMillis()

        var changed = false

        ACTION_CHANNELS.forEach { index ->
            val level =
                inputLevel(
                    current.getOrElse(index) {
                        CHANNEL_NEUTRAL
                    }
                )

            if (level == stableLevels[index]) {
                return@forEach
            }

            if (now - stableChangedAt[index] <
                STICK_DEBOUNCE_MS
            ) {
                return@forEach
            }

            stableLevels[index] = level
            stableChangedAt[index] = now

            if (level != 0) {
                changed = true
            }
        }

        return changed
    }

    private fun inputLevel(value: Int): Int {
        return when {
            value >=
                CHANNEL_HIGH_THRESHOLD -> 1

            value <=
                CHANNEL_LOW_THRESHOLD -> -1

            else -> 0
        }
    }

    /* ============================================================
     * 界面状态格式化
     * ============================================================ */

    private fun formatChannels(
        channels: List<Int>
    ): String {
        return channels.mapIndexed {
                index,
                value ->
            "CH${index + 1}=$value"
        }.joinToString("  ")
    }

    private fun formatDeviceStatus(
        status: StatusEnvelope
    ): String {
        val leaseText =
            if (status.leaseHeld) {
                "lease=" +
                    status.leaseId
                        .ifBlank { "-" } +
                    " owner=" +
                    status.leaseOwner
                        .ifBlank { "-" }
            } else {
                "lease=none"
            }

        return "device: ${status.state} " +
            "${status.detail} " +
            "| epoch=${status.controlEpoch} " +
            "| $leaseText"
    }

    private fun formatSafetyStatus(
        status: StatusEnvelope
    ): String {
        val reason =
            status.safetyReason.ifBlank {
                status.detail.ifBlank { "-" }
            }

        return "safety: ${status.state} | $reason"
    }

    private fun formatWatchdogStatus(
        status: StatusEnvelope
    ): String {
        if (!status.watchdogEnabled) {
            return "watchdog: disabled"
        }

        val active =
            if (status.watchdogActive) {
                "running"
            } else {
                "idle"
            }

        return "watchdog: $active " +
            "timeout=" +
            "${status.watchdogTimeoutSec}s " +
            "lastFeed=" +
            "${status.watchdogLastFeedMs}ms " +
            status.watchdogDetail
                .ifBlank { "-" }
    }

    private fun formatProtocolStatus(
        status: StatusEnvelope
    ): String {
        return "protocol: " +
            "command=${status.commandPort} " +
            "status=${status.statusPort} " +
            "telemetry=" +
            "${status.telemetryPort} " +
            "video=" +
            "${status.videoCamera1Port}/" +
            status.videoCamera2Port
    }

    private fun formatVideoStatus(
        video: VideoStateDto
    ): String {
        if (!video.running) {
            return "video: stopped"
        }

        return "video: running " +
            "camera${video.activeCamera} " +
            videoUrlFor(video)
                .ifBlank { "-" }
    }

    private fun videoUrlFor(
        video: VideoStateDto
    ): String {
        return if (video.activeCamera == 2) {
            video.camera2Url
        } else {
            video.camera1Url
        }
    }

    private fun currentState(): RemoteUiState {
        return synchronized(stateLock) {
            uiState
        }
    }

    private fun updateState(
        reducer:
            (RemoteUiState) -> RemoteUiState
    ) {
        val next =
            synchronized(stateLock) {
                val updated =
                    reducer(uiState)

                uiState = updated
                updated
            }

        observers.forEach { observer ->
            observer(next)
        }
    }

    companion object {
        private const val TAG =
            "SnappyRemoteVM"

        private const val
            CHANNEL_PUMP_INTERVAL_MS = 100L

        /*
         * 摇杆舵位去抖窗口：舵位来回翻转间隔小于该值视为噪声。
         * 实测噪声跳变间隔 2~6ms，正常帧间隔 60ms 以上，取 30ms 兼顾。
         */
        private const val STICK_DEBOUNCE_MS = 30L

        private const val CHANNEL_NEUTRAL =
            1500

        private const val
            CHANNEL_HIGH_THRESHOLD = 1700

        private const val
            CHANNEL_LOW_THRESHOLD = 1300

        private const val
            G20_CHANNEL_MESSAGE_PREFIX =
            "g20-channels-"

        /*
         * 从零开始的动作通道索引：
         * 左右摇杆、GS、L1/L2、相机、SPAY、
         * 暂停、波轮和 R1/R2。
         */
        private val ACTION_CHANNELS =
            intArrayOf(
                0, 1, 2, 3,
                4, 6, 7,
                10, 11, 12, 13, 14, 15
            )
    }
}
