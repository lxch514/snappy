package snappy.remote.app

import android.os.Bundle
import android.view.View
import android.view.Window
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import snappy.protocol.RemoteAction
import snappy.protocol.VideoStateDto
import snappy.remote.g20.G20RemoteDefaults
import snappy.remote.g20.G20SdkAdapter
import snappy.remote.g20.UdpStatusReceiver
import snappy.remote.g20.UdpTelemetryReceiver

class MainActivity : AppCompatActivity() {
    private val permissionManager = PermissionManager()

    private lateinit var settings: Settings
    private lateinit var viewModel: RemoteViewModel

    private var videoPanel: VideoPanel? = null
    private var telemetryPanel: TelemetryPanel? = null
    private var safetyPanel: SafetyPanel? = null
    private var commandPanel: CommandPanel? = null
    private var cameraLabel: TextView? = null
    private var lastVideoRunning: Boolean = false
    private var lastVideoUrl: String = ""
    private var lastVideoCamera: Int = 1

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        supportRequestWindowFeature(Window.FEATURE_NO_TITLE)
        setContentView(R.layout.activity_main)
        enableImmersive()

        settings = Settings(applicationContext)

        permissionManager.requestRuntimePermissions(
            this,
            object : PermissionResultCallback {
                override fun onPermissionGranted() {
                    startRemote()
                }

                override fun onPermissionDenied(deniedPermissions: Array<String>) {
                    startRemote()
                }
            }
        )
    }

    private fun startRemote() {
        if (::viewModel.isInitialized) {
            return
        }

        val root = findViewById<View>(R.id.root)
        val videoAnchor = findViewById<View>(R.id.fpvWidget)
        cameraLabel = findViewById(R.id.tvCameraLabel)

        viewModel = RemoteViewModel(
            robotId = settings.getRobotId().ifBlank { G20RemoteDefaults.ROBOT_ID },
            g20Adapter = G20SdkAdapter(applicationContext),
            statusReceiver = UdpStatusReceiver(context = applicationContext),
            telemetryReceiver = UdpTelemetryReceiver(context = applicationContext)
        )

        telemetryPanel = TelemetryPanel(findViewById(R.id.tvSensorData))

        safetyPanel = SafetyPanel(root) { action, params ->
            if (action == RemoteAction.ControlAcquire) {
                viewModel.acquireAndArm()
            } else {
                viewModel.sendAction(action, params)
            }
        }
        
        commandPanel = CommandPanel(root) { action, params ->
            viewModel.sendAction(action, params)
        }

        viewModel.onG20Channels = { snapshot ->
            runOnUiThread {
                commandPanel?.onG20Snapshot(snapshot)
            }
        }

        videoPanel = VideoPanel(
            anchorView = videoAnchor,
            statusView = null
        ) { playerText ->
            viewModel.setVideoPlayerStatus(playerText)
        }

        viewModel.onVideoState = { video ->
            runOnUiThread {
                handleVideoState(video)
            }
        }

        viewModel.observe { state ->
            runOnUiThread {
                safetyPanel?.render(state)
                commandPanel?.render(state)
                telemetryPanel?.render(state.telemetry, state.telemetryStatus)
            }
        }

        viewModel.setVideoUrl(
            settings.getLastUrl().ifBlank {
                G20RemoteDefaults.DEFAULT_VIDEO_URL_CAMERA1
            }
        )

        cameraLabel?.text = "camera1"
        viewModel.start()
    }

    private fun handleVideoState(video: VideoStateDto) {
    if (video.running) {
        val url =
            if (video.activeCamera == 2) {
                video.camera2Url
            } else {
                video.camera1Url
            }

        settings.setLastUrl(url)
        viewModel.setVideoUrl(url)

        val needsSwitch =
            !lastVideoRunning ||
            url != lastVideoUrl ||
            video.activeCamera != lastVideoCamera

        if (needsSwitch) {
            videoPanel?.switchTo(url)
            lastVideoUrl = url
            lastVideoCamera = video.activeCamera
        }

        lastVideoRunning = true
        cameraLabel?.text = "camera${video.activeCamera}"
        viewModel.setVideoStatus("camera${video.activeCamera}")
    } else {
        if (lastVideoRunning) {
            videoPanel?.stop()
        }

        lastVideoRunning = false
        lastVideoUrl = ""
        lastVideoCamera = 1
        cameraLabel?.text = "camera1"
        viewModel.setVideoStatus("video stopped")
        }
    }

    private fun enableImmersive() {
        window.decorView.systemUiVisibility =
            View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY or
                View.SYSTEM_UI_FLAG_FULLSCREEN or
                View.SYSTEM_UI_FLAG_HIDE_NAVIGATION or
                View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN or
                View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION or
                View.SYSTEM_UI_FLAG_LAYOUT_STABLE
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) {
            enableImmersive()
        }
    }

    override fun onDestroy() {
        try {
            videoPanel?.stop()
        } catch (_: Throwable) {
        }

        if (::viewModel.isInitialized) {
            viewModel.sendAction(RemoteAction.ControlRelease, emptyMap())
            viewModel.stop()
        }

        super.onDestroy()
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)

        val copiedPermissions = Array(permissions.size) { index -> permissions[index] }

        permissionManager.onRequestPermissionsResult(
            requestCode,
            copiedPermissions,
            grantResults
        )
    }
}
