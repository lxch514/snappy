package snappy.protocol

const val SNAPPY_REMOTE_SCHEMA_VERSION = 1

enum class LinkId(val wireValue: String) {
    G20_UDP("g20_udp")
}

enum class PacketKind(val wireValue: String) {
    COMMAND("command"),
    ACK("ack"),
    STATUS("status"),
    TELEMETRY("telemetry")
}

enum class AckStatus {
    ACCEPTED,
    RUNNING,
    SUCCEEDED,
    FAILED,
    ABORTED,
    TIMED_OUT
}

enum class ControlState {
    BOOT,
    SELF_TEST,
    DISARMED,
    ACQUIRED,
    ARMED_IDLE,
    ACTIVE,
    SAFE_HOLD,
    FAULT_LATCHED,
    ESTOP_LATCHED
}

enum class SampleQuality {
    GOOD,
    STALE,
    INVALID,
    UNKNOWN
}