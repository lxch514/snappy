package snappy.remote.app

import android.util.Log
import android.view.View
import android.widget.Button
import snappy.protocol.RemoteAction

class SafetyPanel(
    root: View,
    private val onCommand: (RemoteAction, Map<String, String>) -> Unit
) {
    init {
        val acquireButton = root.findViewById<Button?>(R.id.btnAcquire)
        val faultClearButton = root.findViewById<Button?>(R.id.btnFaultClear)
        val safeHoldButton = root.findViewById<Button?>(R.id.btnSafeHold)

        Log.i(
            TAG,
            "binding buttons acquire=${acquireButton != null} faultClear=${faultClearButton != null} safeHold=${safeHoldButton != null}"
        )

        if (acquireButton == null) {
            Log.e(TAG, "btnAcquire findViewById returned null")
        }
        if (faultClearButton == null) {
            Log.e(TAG, "btnFaultClear findViewById returned null")
        }
        if (safeHoldButton == null) {
            Log.e(TAG, "btnSafeHold findViewById returned null")
        }

        acquireButton?.setOnClickListener {
            Log.i(TAG, "button click action=${RemoteAction.ControlAcquire.wireValue}")
            onCommand(RemoteAction.ControlAcquire, emptyMap())
        }

        faultClearButton?.setOnClickListener {
            Log.i(TAG, "button click action=${RemoteAction.FaultClear.wireValue}")
            onCommand(RemoteAction.FaultClear, emptyMap())
        }

        safeHoldButton?.setOnClickListener {
            Log.i(TAG, "button click action=${RemoteAction.ControlSafeHold.wireValue}")
            onCommand(RemoteAction.ControlSafeHold, emptyMap())
        }
    }

    fun render(state: RemoteUiState) {
        // The redesigned screen intentionally hides protocol/status chatter.
    }

    companion object {
        private const val TAG = "SnappySafety"
    }
}