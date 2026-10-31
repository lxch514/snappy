package snappy.remote.g20

enum class LinkHealth {
    DISCONNECTED,
    CONNECTING,
    ONLINE,
    STALE,
    FAULT
}

data class LinkState(
    val health: LinkHealth = LinkHealth.DISCONNECTED,
    val lastTxMs: Long = 0,
    val lastRxMs: Long = 0,
    val lastError: String = "",
    val statusText: String = "not connected"
)

class LinkStateTracker(
    private val staleTimeoutMs: Long = 1000
) {
    private var state = LinkState()

    @Synchronized
    fun connecting() {
        state = state.copy(
            health = LinkHealth.CONNECTING,
            statusText = "connecting"
        )
    }

    @Synchronized
    fun markTx(nowMs: Long = System.currentTimeMillis()) {
        state = state.copy(
            health = if (state.health == LinkHealth.FAULT) LinkHealth.FAULT else LinkHealth.ONLINE,
            lastTxMs = nowMs,
            statusText = "tx ok"
        )
    }

    @Synchronized
    fun markRx(nowMs: Long = System.currentTimeMillis()) {
        state = state.copy(
            health = LinkHealth.ONLINE,
            lastRxMs = nowMs,
            lastError = "",
            statusText = "online"
        )
    }

    @Synchronized
    fun fault(error: String) {
        state = state.copy(
            health = LinkHealth.FAULT,
            lastError = error,
            statusText = error
        )
    }

    @Synchronized
    fun disconnect() {
        state = LinkState()
    }

    @Synchronized
    fun snapshot(nowMs: Long = System.currentTimeMillis()): LinkState {
        if (state.health == LinkHealth.ONLINE &&
            state.lastRxMs > 0 &&
            nowMs - state.lastRxMs > staleTimeoutMs
        ) {
            state = state.copy(
                health = LinkHealth.STALE,
                statusText = "stale"
            )
        }
        return state
    }
}