/**
 * @file CommandPanel.kt
 * @brief 遥控器右侧操作面板
 */

package snappy.remote.app

import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.view.View
import android.widget.Button
import android.widget.SeekBar
import android.widget.TextView
import java.util.Locale
import kotlin.math.ceil
import kotlin.math.roundToInt
import snappy.protocol.RemoteAction
import snappy.remote.g20.G20ChannelSnapshot

class CommandPanel(
    root: View,
    private val onCommand: (RemoteAction, Map<String, String>) -> Unit
) {
    private enum class ClimberMode {
        STOPPED,
        ASCEND,
        DESCEND
    }

    private enum class G20PushTarget {
        NONE,
        BRUSH,
        SCRAPER
    }

    private enum class G20PushSetup {
        IDLE,
        EXTEND,
        RETRACT
    }

    private val mainHandler =
        Handler(Looper.getMainLooper())

    private var pendingClimberSend: Runnable? = null

    private val brushPushSeek: SeekBar? =
        root.findViewById(R.id.seekBrushPushDistance)

    private val brushPushValue: TextView? =
        root.findViewById(R.id.tvBrushPushDistanceValue)

    private val brushPushSecondsSeek: SeekBar? =
        root.findViewById(R.id.seekBrushPushSeconds)

    private val brushPushSecondsValue: TextView? =
        root.findViewById(R.id.tvBrushPushSecondsValue)

    private val scraperPushSeek: SeekBar? =
        root.findViewById(R.id.seekScraperPushDistance)

    private val scraperPushValue: TextView? =
        root.findViewById(R.id.tvScraperPushDistanceValue)

    private val scraperPushSecondsSeek: SeekBar? =
        root.findViewById(R.id.seekScraperPushSeconds)

    private val scraperPushSecondsValue: TextView? =
        root.findViewById(R.id.tvScraperPushSecondsValue)

    private val fanSeek: SeekBar? =
        root.findViewById(R.id.seekFanSpeed)

    private val fanValue: TextView? =
        root.findViewById(R.id.tvFanSpeedValue)

    private val climberSeek: SeekBar? =
        root.findViewById(R.id.seekClimberSpeed)

    private val climberValue: TextView? =
        root.findViewById(R.id.tvClimberSpeedValue)

    private var fanRunning = false
    private var climberMode = ClimberMode.STOPPED
    private var climberTargetRpm = 0.0

    private var brushPushExtensionMm = 0
    private var scraperPushExtensionMm = 0

    private var renderingFromState = false
    private var climberUserTracking = false
    private var climberStatusHoldUntilMs = 0L
    private var fanUserTracking = false
    private var fanStatusHoldUntilMs = 0L
    private var syncingPushPair = false
    private var pendingG20PushReset: Runnable? = null

    private var g20Initialized = false
    private var g20Target = G20PushTarget.NONE
    private var g20Setup = G20PushSetup.IDLE
    private var g20PreviousGs = 0
    private var g20PreviousX = 0
    private var g20PreviousY = 0
    private var g20LastAdjustMs = 0L
    private var brushPushWasMoving = false
    private var scraperPushWasMoving = false

    init {
        brushPushSeek?.max = PUSH_DISTANCE_MAX_MM
        brushPushSecondsSeek?.max =
            BRUSH_PUSH_MAX_SECONDS

        scraperPushSeek?.max =
            PUSH_DISTANCE_MAX_MM

        scraperPushSecondsSeek?.max =
            SCRAPER_PUSH_MAX_SECONDS

        fanSeek?.max = FAN_PROGRESS_MAX
        climberSeek?.max = CLIMBER_PROGRESS_MAX

        setBrushPushSeconds(0)
        setScraperPushSeconds(0)

        fanSeek?.progress = 0
        updateFanValue(0)

        climberSeek?.progress =
            climberProgressFor(climberTargetRpm)

        updateClimberValue(climberTargetRpm)

        root.button(R.id.btnBrushForward)?.setOnClickListener {
            onCommand(
                RemoteAction.BrushForward,
                mapOf(
                    "speed_raw" to
                        BRUSH_MAX_SPEED_RAW.toString()
                )
            )
        }

        root.button(R.id.btnBrushReverse)?.setOnClickListener {
            onCommand(
                RemoteAction.BrushReverse,
                mapOf(
                    "speed_raw" to
                        BRUSH_MAX_SPEED_RAW.toString()
                )
            )
        }

        root.button(R.id.btnBrushStop)?.setOnClickListener {
            onCommand(
                RemoteAction.BrushStop,
                emptyMap()
            )
        }

        brushPushSeek?.setOnSeekBarChangeListener(
            simpleDistanceListener {
                updateBrushPushDistanceValue()
            }
        )

        brushPushSecondsSeek?.setOnSeekBarChangeListener(
            simpleSecondsListener {
                setBrushPushSeconds(
                    brushPushSecondsSeek?.progress ?: 0
                )
            }
        )

        scraperPushSeek?.setOnSeekBarChangeListener(
            simpleDistanceListener {
                updateScraperPushDistanceValue()
            }
        )

        scraperPushSecondsSeek?.setOnSeekBarChangeListener(
            simpleSecondsListener {
                setScraperPushSeconds(
                    scraperPushSecondsSeek?.progress ?: 0
                )
            }
        )

        root.button(R.id.btnBrushPushExtend)?.setOnClickListener {
            sendBrushPushTimed(true)
        }

        root.button(R.id.btnBrushPushRetract)?.setOnClickListener {
            sendBrushPushTimed(false)
        }

        root.button(R.id.btnBrushPushStop)?.setOnClickListener {
            onCommand(
                RemoteAction.BrushPushStop,
                emptyMap()
            )
        }

        root.button(R.id.btnScraperPushExtend)?.setOnClickListener {
            sendScraperPushTimed(true)
        }

        root.button(R.id.btnScraperPushRetract)?.setOnClickListener {
            sendScraperPushTimed(false)
        }

        root.button(R.id.btnScraperPushStop)?.setOnClickListener {
            onCommand(
                RemoteAction.ScraperPushStop,
                emptyMap()
            )
        }

        root.button(R.id.btnFanStart)?.setOnClickListener {
            fanRunning = true
            fanUserTracking = false

            fanSeek?.progress = 0
            updateFanValue(FAN_BASE_RAW)

            holdFanStatus()

            onCommand(
                RemoteAction.FanPrepare,
                emptyMap()
            )

            sendFanSpeed()
        }

        root.button(R.id.btnFanStop)?.setOnClickListener {
            fanRunning = false
            fanUserTracking = false

            holdFanStatus()
            updateFanValue(0)

            onCommand(
                RemoteAction.FanStop,
                emptyMap()
            )
        }

        fanSeek?.setOnSeekBarChangeListener(
            object : SeekBar.OnSeekBarChangeListener {
                override fun onProgressChanged(
                    seekBar: SeekBar?,
                    progress: Int,
                    fromUser: Boolean
                ) {
                    val raw = currentFanRaw()

                    if (fromUser &&
                        !renderingFromState) {
                        holdFanStatus()
                    }

                    updateFanValue(raw)

                    if (fromUser &&
                        !renderingFromState &&
                        fanRunning) {
                        sendFanSpeed()
                    }
                }

                override fun onStartTrackingTouch(
                    seekBar: SeekBar?
                ) {
                    fanUserTracking = true
                    holdFanStatus()
                }

                override fun onStopTrackingTouch(
                    seekBar: SeekBar?
                ) {
                    fanUserTracking = false
                    holdFanStatus()

                    if (!renderingFromState &&
                        fanRunning) {
                        sendFanSpeed()
                    }
                }
            }
        )

        root.button(R.id.btnClimberAscend)?.setOnClickListener {
            startClimber(ClimberMode.ASCEND)
        }

        root.button(R.id.btnClimberDescend)?.setOnClickListener {
            startClimber(ClimberMode.DESCEND)
        }

        root.button(R.id.btnClimberStop)?.setOnClickListener {
            stopClimber()
        }

        climberSeek?.setOnSeekBarChangeListener(
            object : SeekBar.OnSeekBarChangeListener {
                override fun onProgressChanged(
                    seekBar: SeekBar?,
                    progress: Int,
                    fromUser: Boolean
                ) {
                    val speed = currentClimberRpm()

                    if (fromUser &&
                        !renderingFromState) {
                        climberTargetRpm = speed
                    }

                    updateClimberValue(
                        climberTargetRpm
                    )

                    if (fromUser &&
                        !renderingFromState &&
                        climberMode !=
                            ClimberMode.STOPPED) {
                        scheduleClimberSend()
                    }
                }

                override fun onStartTrackingTouch(
                    seekBar: SeekBar?
                ) {
                    climberUserTracking = true
                    holdClimberStatus()
                }

                override fun onStopTrackingTouch(
                    seekBar: SeekBar?
                ) {
                    climberUserTracking = false
                    climberTargetRpm =
                        currentClimberRpm()

                    updateClimberValue(
                        climberTargetRpm
                    )

                    if (!renderingFromState &&
                        climberMode !=
                            ClimberMode.STOPPED) {
                        scheduleClimberSend()
                    }
                }
            }
        )
    }

    fun render(state: RemoteUiState) {
        val actuators =
            state.status?.actuators ?: state.actuators

        renderingFromState = true

        try {
            brushPushExtensionMm =
                actuators.brushPushExtensionMm
                    .coerceIn(
                        0,
                        PUSH_DISTANCE_MAX_MM
                    )

            scraperPushExtensionMm =
                actuators.scraperPushExtensionMm
                    .coerceIn(
                        0,
                        PUSH_DISTANCE_MAX_MM
                    )

            updateBrushPushDistanceValue()
            updateScraperPushDistanceValue()

            val brushPushMoving = pushMotionActive(
                actuators.brushPushState,
                actuators.brushPushRemainingMs
            )
            val scraperPushMoving = pushMotionActive(
                actuators.scraperPushState,
                actuators.scraperPushRemainingMs
            )

            if (brushPushWasMoving && !brushPushMoving) {
                setBrushPushSeconds(0)
            }
            if (scraperPushWasMoving && !scraperPushMoving) {
                setScraperPushSeconds(0)
            }

            brushPushWasMoving = brushPushMoving
            scraperPushWasMoving = scraperPushMoving

            if (!shouldHoldFanUi()) {
                val fanReportedRaw =
                    maxOf(
                        actuators.fanLeftRaw,
                        actuators.fanRightRaw
                    )

                fanRunning =
                    actuators.fanActive ||
                        fanReportedRaw > 0

                if (fanReportedRaw > 0) {
                    if (!fanUserTracking) {
                        fanSeek?.progress =
                            fanProgressFor(
                                fanReportedRaw
                            )
                    }

                    updateFanValue(
                        fanReportedRaw
                    )
                } else {
                    updateFanValue(0)
                }
            } else {
                updateFanValue(
                    if (fanRunning) {
                        currentFanRaw()
                    } else {
                        0
                    }
                )
            }

            if (!shouldHoldClimberUi()) {
                val reportedMode =
                    when (
                        actuators.climberState
                            .lowercase(Locale.US)
                    ) {
                        "ascend",
                        "ascending" ->
                            ClimberMode.ASCEND

                        "descend",
                        "descending" ->
                            ClimberMode.DESCEND

                        else ->
                            ClimberMode.STOPPED
                    }

                if (reportedMode !=
                    ClimberMode.STOPPED) {
                    climberMode = reportedMode

                    if (climberTargetRpm <= 0.0) {
                        climberTargetRpm =
                            CLIMBER_BASE_RPM
                    }

                    if (actuators.climberSpeedRpm >
                        0.0) {
                        val reportedSpeed =
                            actuators.climberSpeedRpm
                                .coerceIn(
                                    CLIMBER_BASE_RPM,
                                    CLIMBER_MAX_RPM
                                )

                        val reportedProgress =
                            climberProgressFor(
                                reportedSpeed
                            )

                        climberSeek?.progress =
                            reportedProgress

                        climberTargetRpm =
                            CLIMBER_BASE_RPM +
                                reportedProgress *
                                CLIMBER_STEP_RPM
                    }
                } else {
                    climberMode =
                        ClimberMode.STOPPED
                    climberTargetRpm = 0.0
                    climberSeek?.progress = 0
                }

                updateClimberValue(
                    climberTargetRpm
                )
            } else {
                updateClimberValue(
                    climberTargetRpm
                )
            }
        } finally {
            renderingFromState = false
        }
    }

    private fun pushMotionActive(
        state: String,
        remainingMs: Long
    ): Boolean {
        val motion = state.lowercase(Locale.US)
        return remainingMs > 0L ||
            motion == "extend" ||
            motion == "retract"
    }

    private fun startClimber(
        mode: ClimberMode
    ) {
        cancelPendingClimberSend()

        climberMode = mode
        climberUserTracking = false

        climberTargetRpm =
            currentClimberRpm().coerceIn(
                CLIMBER_BASE_RPM,
                CLIMBER_MAX_RPM
            )

        climberSeek?.progress =
            climberProgressFor(
                climberTargetRpm
            )

        updateClimberValue(
            climberTargetRpm
        )

        holdClimberStatus()

        sendClimberSpeed(
            mode = mode,
            speed = climberTargetRpm
        )
    }

    private fun stopClimber() {
        cancelPendingClimberSend()

        climberMode =
            ClimberMode.STOPPED
        climberUserTracking = false
        climberTargetRpm = 0.0

        climberSeek?.progress = 0
        updateClimberValue(0.0)
        holdClimberStatus()

        onCommand(
            RemoteAction.ClimberStop,
            emptyMap()
        )
    }

    private fun scheduleClimberSend() {
        cancelPendingClimberSend()
        holdClimberStatus()

        val targetMode = climberMode
        val targetSpeed =
            currentClimberRpm().coerceIn(
                CLIMBER_BASE_RPM,
                CLIMBER_MAX_RPM
            )

        climberTargetRpm = targetSpeed
        updateClimberValue(
            climberTargetRpm
        )

        val runnable = Runnable {
            pendingClimberSend = null

            if (!renderingFromState &&
                climberMode == targetMode &&
                targetMode !=
                    ClimberMode.STOPPED) {
                sendClimberSpeed(
                    mode = targetMode,
                    speed = targetSpeed
                )

                holdClimberStatus()
            }
        }

        pendingClimberSend = runnable

        mainHandler.postDelayed(
            runnable,
            CLIMBER_SETTLE_DELAY_MS
        )
    }

    private fun cancelPendingClimberSend() {
        pendingClimberSend?.let(
            mainHandler::removeCallbacks
        )

        pendingClimberSend = null
    }

    private fun shouldHoldFanUi(): Boolean {
        return fanUserTracking ||
            SystemClock.elapsedRealtime() <
                fanStatusHoldUntilMs
    }

    private fun holdFanStatus() {
        fanStatusHoldUntilMs =
            SystemClock.elapsedRealtime() +
                FAN_STATUS_HOLD_MS
    }

    private fun shouldHoldClimberUi(): Boolean {
        return climberUserTracking ||
            pendingClimberSend != null ||
            SystemClock.elapsedRealtime() <
                climberStatusHoldUntilMs
    }

    private fun holdClimberStatus() {
        climberStatusHoldUntilMs =
            SystemClock.elapsedRealtime() +
                CLIMBER_STATUS_HOLD_MS
    }

    private fun sendClimberSpeed(
        mode: ClimberMode = climberMode,
        speed: Double = climberTargetRpm
    ) {
        val safeSpeed =
            speed.coerceIn(
                CLIMBER_BASE_RPM,
                CLIMBER_MAX_RPM
            )

        climberTargetRpm = safeSpeed

        val params = mapOf(
            "speed_rpm" to String.format(
                Locale.US,
                "%.0f",
                safeSpeed
            )
        )

        when (mode) {
            ClimberMode.ASCEND -> {
                onCommand(
                    RemoteAction.ClimberAscend,
                    params
                )
            }

            ClimberMode.DESCEND -> {
                onCommand(
                    RemoteAction.ClimberDescend,
                    params
                )
            }

            ClimberMode.STOPPED -> Unit
        }
    }

    private fun sendBrushPushTimed(
        physicalExtend: Boolean
    ) {
        val requestedDistanceMm =
            currentBrushPushDistanceMm()

        val availableDistanceMm =
            if (physicalExtend) {
                (
                    PUSH_DISTANCE_MAX_MM -
                        brushPushExtensionMm
                    ).coerceAtLeast(0)
            } else {
                brushPushExtensionMm
                    .coerceAtLeast(0)
            }

        val distanceMm =
            requestedDistanceMm.coerceAtMost(
                availableDistanceMm
            )

        if (distanceMm <= 0) {
            onCommand(
                RemoteAction.BrushPushStop,
                emptyMap()
            )
            return
        }

        val durationMs =
            durationMsForDistance(
                distanceMm,
                BRUSH_PUSH_SPEED_MM_PER_S
            )

        onCommand(
            if (physicalExtend) {
                RemoteAction.BrushPushExtendTimed
            } else {
                RemoteAction.BrushPushRetractTimed
            },
            mapOf(
                "distance_mm" to
                    distanceMm.toString(),
                "duration_ms" to
                    durationMs.toString()
            )
        )
    }

    private fun sendScraperPushTimed(
        physicalExtend: Boolean
    ) {
        val requestedDistanceMm =
            currentScraperPushDistanceMm()

        val availableDistanceMm =
            if (physicalExtend) {
                (
                    PUSH_DISTANCE_MAX_MM -
                        scraperPushExtensionMm
                    ).coerceAtLeast(0)
            } else {
                scraperPushExtensionMm
                    .coerceAtLeast(0)
            }

        val distanceMm =
            requestedDistanceMm.coerceAtMost(
                availableDistanceMm
            )

        if (distanceMm <= 0) {
            onCommand(
                RemoteAction.ScraperPushStop,
                emptyMap()
            )
            return
        }

        val durationMs =
            durationMsForDistance(
                distanceMm,
                SCRAPER_PUSH_SPEED_MM_PER_S
            )

        onCommand(
            if (physicalExtend) {
                RemoteAction.ScraperPushExtendTimed
            } else {
                RemoteAction.ScraperPushRetractTimed
            },
            mapOf(
                "distance_mm" to
                    distanceMm.toString(),
                "duration_ms" to
                    durationMs.toString()
            )
        )
    }

    private fun durationMsForDistance(
        distanceMm: Int,
        speedMmPerS: Double
    ): Long {
        if (distanceMm <= 0 ||
            speedMmPerS <= 0.0) {
            return 0L
        }

        return ceil(
            distanceMm * 1000.0 /
                speedMmPerS
        ).toLong()
    }

    private fun currentBrushPushDistanceMm(): Int {
        return brushPushSeek
            ?.progress
            ?.coerceIn(
                0,
                PUSH_DISTANCE_MAX_MM
            )
            ?: 0
    }

    private fun currentBrushPushSeconds(): Int {
        return brushPushSecondsSeek
            ?.progress
            ?.coerceIn(
                0,
                BRUSH_PUSH_MAX_SECONDS
            )
            ?: 0
    }

    private fun currentScraperPushDistanceMm(): Int {
        return scraperPushSeek
            ?.progress
            ?.coerceIn(
                0,
                PUSH_DISTANCE_MAX_MM
            )
            ?: 0
    }

    private fun currentScraperPushSeconds(): Int {
        return scraperPushSecondsSeek
            ?.progress
            ?.coerceIn(
                0,
                SCRAPER_PUSH_MAX_SECONDS
            )
            ?: 0
    }

    private fun setBrushPushSeconds(
        seconds: Int
    ) {
        val safeSeconds =
            seconds.coerceIn(
                0,
                BRUSH_PUSH_MAX_SECONDS
            )

        syncingPushPair = true

        try {
            brushPushSecondsSeek?.progress =
                safeSeconds

            brushPushSeek?.progress =
                distanceForSeconds(
                    safeSeconds,
                    BRUSH_PUSH_SPEED_MM_PER_S
                )
        } finally {
            syncingPushPair = false
        }

        updateBrushPushDistanceValue()
        updateBrushPushSecondsValue()
    }

    private fun setScraperPushSeconds(
        seconds: Int
    ) {
        val safeSeconds =
            seconds.coerceIn(
                0,
                SCRAPER_PUSH_MAX_SECONDS
            )

        syncingPushPair = true

        try {
            scraperPushSecondsSeek?.progress =
                safeSeconds

            scraperPushSeek?.progress =
                distanceForSeconds(
                    safeSeconds,
                    SCRAPER_PUSH_SPEED_MM_PER_S
                )
        } finally {
            syncingPushPair = false
        }

        updateScraperPushDistanceValue()
        updateScraperPushSecondsValue()
    }

    private fun distanceForSeconds(
        seconds: Int,
        speedMmPerS: Double
    ): Int {
        return (
            seconds * speedMmPerS
        )
            .roundToInt()
            .coerceIn(
                0,
                PUSH_DISTANCE_MAX_MM
            )
    }

    private fun secondsForDistance(
        distanceMm: Int,
        speedMmPerS: Double,
        maxSeconds: Int
    ): Int {
        if (distanceMm <= 0 ||
            speedMmPerS <= 0.0) {
            return 0
        }

        return (
            distanceMm / speedMmPerS
        )
            .roundToInt()
            .coerceIn(
                0,
                maxSeconds
            )
    }

    private fun updateBrushPushDistanceValue() {
        brushPushValue?.text =
            String.format(
                Locale.US,
                "%dmm/%dmm",
                currentBrushPushDistanceMm(),
                brushPushExtensionMm
            )
    }

    private fun updateBrushPushSecondsValue() {
        brushPushSecondsValue?.text =
            String.format(
                Locale.US,
                "%d s",
                currentBrushPushSeconds()
            )
    }

    private fun updateScraperPushDistanceValue() {
        scraperPushValue?.text =
            String.format(
                Locale.US,
                "%dmm/%dmm",
                currentScraperPushDistanceMm(),
                scraperPushExtensionMm
            )
    }

    private fun updateScraperPushSecondsValue() {
        scraperPushSecondsValue?.text =
            String.format(
                Locale.US,
                "%d s",
                currentScraperPushSeconds()
            )
    }

    private fun sendFanSpeed() {
        val raw = currentFanRaw()

        onCommand(
            RemoteAction.FanBothSet,
            mapOf(
                "left_raw" to raw.toString(),
                "right_raw" to raw.toString()
            )
        )
    }

    private fun currentFanRaw(): Int {
        val progress =
            (fanSeek?.progress ?: 0)
                .coerceIn(
                    0,
                    FAN_PROGRESS_MAX
                )

        if (progress >= FAN_PROGRESS_MAX) {
            return FAN_MAX_RAW
        }

        return (
            FAN_BASE_RAW +
                progress * FAN_STEP_RAW
        ).coerceAtMost(FAN_MAX_RAW)
    }

    private fun fanProgressFor(raw: Int): Int {
        if (raw <= FAN_BASE_RAW) {
            return 0
        }

        if (raw >= FAN_MAX_RAW) {
            return FAN_PROGRESS_MAX
        }

        val progress =
            (
                raw -
                    FAN_BASE_RAW +
                    FAN_STEP_RAW -
                    1
            ) / FAN_STEP_RAW

        return progress.coerceIn(
            0,
            FAN_PROGRESS_MAX - 1
        )
    }

    private fun updateFanValue(raw: Int) {
        fanValue?.text =
            raw.coerceIn(
                0,
                FAN_MAX_RAW
            ).toString()
    }

    private fun currentClimberRpm(): Double {
        val progress =
            climberSeek?.progress ?: 0

        return CLIMBER_BASE_RPM +
            progress.coerceIn(
                0,
                CLIMBER_PROGRESS_MAX
            ) * CLIMBER_STEP_RPM
    }

    private fun climberProgressFor(
        speedRpm: Double
    ): Int {
        if (speedRpm <= CLIMBER_BASE_RPM) {
            return 0
        }

        return (
            (speedRpm - CLIMBER_BASE_RPM) /
                CLIMBER_STEP_RPM
        )
            .roundToInt()
            .coerceIn(
                0,
                CLIMBER_PROGRESS_MAX
            )
    }

    private fun updateClimberValue(
        speedRpm: Double
    ) {
        climberValue?.text =
            String.format(
                Locale.US,
                "%.0f",
                speedRpm.coerceIn(
                    0.0,
                    CLIMBER_MAX_RPM
                )
            )
    }

    fun onG20Snapshot(
        snapshot: G20ChannelSnapshot
    ) {
        val channels =
            snapshot.normalizedChannels()

        val gs =
            levelAt(
                channels,
                G20_GS_INDEX
            )

        val x =
            levelAt(
                channels,
                G20_LEFT_STICK_X_INDEX
            )

        val y =
            levelAt(
                channels,
                G20_LEFT_STICK_Y_INDEX
            )

        val now =
            SystemClock.elapsedRealtime()

        if (!g20Initialized) {
            g20Initialized = true
            g20PreviousGs = gs
            g20PreviousX = x
            g20PreviousY = y
            selectG20PushTarget(gs)
            return
        }

        if (gs != g20PreviousGs) {
            selectG20PushTarget(gs)
        }

        if (
            g20Target != G20PushTarget.NONE &&
            g20Setup != G20PushSetup.IDLE &&
            x != 0 &&
            (
                x != g20PreviousX ||
                    now - g20LastAdjustMs >=
                        G20_REPEAT_INTERVAL_MS
            )
        ) {
            adjustG20PushSeconds(
                if (x < 0) -1 else 1
            )

            g20LastAdjustMs = now
        }

        if (y != g20PreviousY && y != 0) {
            handleG20PushY(y)
        }

        g20PreviousGs = gs
        g20PreviousX = x
        g20PreviousY = y
    }

    private fun selectG20PushTarget(
        level: Int
    ) {
        pendingG20PushReset?.let {
            mainHandler.removeCallbacks(it)
        }

        pendingG20PushReset = null

        g20Target =
            when {
                level > 0 ->
                    G20PushTarget.SCRAPER

                level < 0 ->
                    G20PushTarget.BRUSH

                else ->
                    G20PushTarget.NONE
            }

        g20Setup =
            G20PushSetup.IDLE

        g20LastAdjustMs =
            SystemClock.elapsedRealtime()

        when (g20Target) {
            G20PushTarget.BRUSH ->
                setBrushPushSeconds(0)

            G20PushTarget.SCRAPER ->
                setScraperPushSeconds(0)

            G20PushTarget.NONE -> {
                setBrushPushSeconds(0)
                setScraperPushSeconds(0)
            }
        }
    }

    private fun handleG20PushY(
        level: Int
    ) {
        val requestedSetup =
            when {
                level > 0 ->
                    G20PushSetup.EXTEND

                level < 0 ->
                    G20PushSetup.RETRACT

                else ->
                    return
            }

        if (g20Setup != requestedSetup) {
            g20Setup = requestedSetup
            setG20PushSeconds(0)
            return
        }

        val seconds =
            currentG20PushSeconds()

        if (seconds <= 0) {
            return
        }

        g20Setup =
            G20PushSetup.IDLE

        scheduleG20PushReset(seconds)
    }

    private fun scheduleG20PushReset(
        seconds: Int
    ) {
        pendingG20PushReset?.let {
            mainHandler.removeCallbacks(it)
        }

        val targetAtStart = g20Target
        val secondsAtStart = seconds

        val distanceMm =
            when (targetAtStart) {
                G20PushTarget.BRUSH ->
                    distanceForSeconds(
                        seconds,
                        BRUSH_PUSH_SPEED_MM_PER_S
                    )

                G20PushTarget.SCRAPER ->
                    distanceForSeconds(
                        seconds,
                        SCRAPER_PUSH_SPEED_MM_PER_S
                    )

                G20PushTarget.NONE ->
                    0
            }

        val speedMmPerSecond =
            when (targetAtStart) {
                G20PushTarget.BRUSH ->
                    BRUSH_PUSH_SPEED_MM_PER_S

                G20PushTarget.SCRAPER ->
                    SCRAPER_PUSH_SPEED_MM_PER_S

                G20PushTarget.NONE ->
                    1.0
            }

        val delayMs =
            durationMsForDistance(
                distanceMm,
                speedMmPerSecond
            ) + G20_PUSH_RESET_MARGIN_MS

        val resetTask = Runnable {
            if (
                g20Target == targetAtStart &&
                g20Setup == G20PushSetup.IDLE &&
                currentG20PushSeconds() ==
                    secondsAtStart
            ) {
                setG20PushSeconds(0)
            }

            pendingG20PushReset = null
        }

        pendingG20PushReset = resetTask

        mainHandler.postDelayed(
            resetTask,
            delayMs
        )
    }

    private fun adjustG20PushSeconds(
        direction: Int
    ) {
        when (g20Target) {
            G20PushTarget.BRUSH -> {
                setBrushPushSeconds(
                    currentBrushPushSeconds() +
                        direction
                )
            }

            G20PushTarget.SCRAPER -> {
                setScraperPushSeconds(
                    currentScraperPushSeconds() +
                        direction
                )
            }

            G20PushTarget.NONE -> Unit
        }
    }

    private fun currentG20PushSeconds(): Int {
        return when (g20Target) {
            G20PushTarget.BRUSH ->
                currentBrushPushSeconds()

            G20PushTarget.SCRAPER ->
                currentScraperPushSeconds()

            G20PushTarget.NONE ->
                0
        }
    }

    private fun setG20PushSeconds(
        seconds: Int
    ) {
        when (g20Target) {
            G20PushTarget.BRUSH ->
                setBrushPushSeconds(seconds)

            G20PushTarget.SCRAPER ->
                setScraperPushSeconds(seconds)

            G20PushTarget.NONE ->
                Unit
        }
    }

    private fun levelAt(
        channels: List<Int>,
        index: Int
    ): Int {
        val value =
            channels.getOrNull(index)
                ?: return 0

        return when {
            value >= G20_HIGH_THRESHOLD -> 1

            value <= G20_LOW_THRESHOLD -> -1

            else -> 0
        }
    }

    private fun simpleDistanceListener(
        onChanged: () -> Unit
    ): SeekBar.OnSeekBarChangeListener {
        return object :
            SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(
                seekBar: SeekBar?,
                progress: Int,
                fromUser: Boolean
            ) {
                onChanged()
            }

            override fun onStartTrackingTouch(
                seekBar: SeekBar?
            ) = Unit

            override fun onStopTrackingTouch(
                seekBar: SeekBar?
            ) = Unit
        }
    }

    private fun simpleSecondsListener(
        onChanged: () -> Unit
    ): SeekBar.OnSeekBarChangeListener {
        return object :
            SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(
                seekBar: SeekBar?,
                progress: Int,
                fromUser: Boolean
            ) {
                onChanged()
            }

            override fun onStartTrackingTouch(
                seekBar: SeekBar?
            ) = Unit

            override fun onStopTrackingTouch(
                seekBar: SeekBar?
            ) = Unit
        }
    }

    private fun View.button(
        id: Int
    ): Button? =
        findViewById(id)

    companion object {
        private const val BRUSH_MAX_SPEED_RAW = 15

        private const val PUSH_DISTANCE_MAX_MM = 300
        private const val BRUSH_PUSH_SPEED_MM_PER_S = 33.0
        private const val SCRAPER_PUSH_SPEED_MM_PER_S = 30.0
        private const val BRUSH_PUSH_MAX_SECONDS = 10
        private const val SCRAPER_PUSH_MAX_SECONDS = 10

        private const val BRUSH_PUSH_MAX_DURATION_MS = 9100L
        private const val SCRAPER_PUSH_MAX_DURATION_MS = 10000L

        private const val G20_LEFT_STICK_X_INDEX = 3
        private const val G20_LEFT_STICK_Y_INDEX = 2
        private const val G20_GS_INDEX = 4
        private const val G20_LOW_THRESHOLD = 1300
        private const val G20_HIGH_THRESHOLD = 1700
        private const val G20_REPEAT_INTERVAL_MS = 500L
        private const val G20_PUSH_RESET_MARGIN_MS = 250L

        private const val FAN_BASE_RAW = 100
        private const val FAN_STEP_RAW = 400
        private const val FAN_MAX_RAW = 8191
        private const val FAN_PROGRESS_MAX = 21
        private const val FAN_STATUS_HOLD_MS = 1200L

        private const val CLIMBER_BASE_RPM = 500.0
        private const val CLIMBER_STEP_RPM = 100.0
        private const val CLIMBER_MAX_RPM = 3000.0
        private const val CLIMBER_PROGRESS_MAX = 25

        private const val CLIMBER_SETTLE_DELAY_MS = 500L
        private const val CLIMBER_STATUS_HOLD_MS = 1200L
    }
}