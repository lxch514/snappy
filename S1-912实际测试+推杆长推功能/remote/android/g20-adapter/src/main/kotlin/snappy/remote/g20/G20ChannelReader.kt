package snappy.remote.g20

interface G20ChannelReader {
    fun start(listener: (G20ChannelSnapshot) -> Unit): Boolean
    fun stop()
}