package snappy.remote.g20

class NullG20Adapter : G20Adapter {
    override val available: Boolean = false
    override val statusText: String = "G20: unavailable"

    override fun start(listener: (G20ChannelSnapshot) -> Unit): Boolean {
        return false
    }

    override fun stop() {
    }
}