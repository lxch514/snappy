package snappy.remote.g20

class FakeG20Adapter : G20Adapter {
    @Volatile
    override var available: Boolean = false
        private set

    @Volatile
    override var statusText: String = "Fake G20: stopped"
        private set

    private var listener: ((G20ChannelSnapshot) -> Unit)? = null

    override fun start(listener: (G20ChannelSnapshot) -> Unit): Boolean {
        this.listener = listener
        available = true
        statusText = "Fake G20: running"
        return true
    }

    fun emit(values: List<Int>) {
        if (!available) return
        listener?.invoke(G20ChannelSnapshot(values = values, deviceName = "fake"))
    }

    override fun stop() {
        available = false
        statusText = "Fake G20: stopped"
        listener = null
    }
}