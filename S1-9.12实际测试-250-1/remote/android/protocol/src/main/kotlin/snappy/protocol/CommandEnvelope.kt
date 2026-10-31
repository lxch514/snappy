package snappy.protocol

data class CommandEnvelope(
    val schemaVersion: Int = SNAPPY_REMOTE_SCHEMA_VERSION,
    val kind: String = PacketKind.COMMAND.wireValue,
    val robotId: String,
    val linkId: String = LinkId.G20_UDP.wireValue,
    val sourceId: String,
    val sessionId: String,
    val messageId: String,
    val sequence: Long,
    val issuedAtMs: Long,
    val ttlMs: Long = 250,
    val controlEpoch: Long = 0,
    val leaseId: String? = null,
    val action: String,
    val params: Map<String, String> = emptyMap()
) {
    companion object {
        fun of(
            robotId: String,
            sourceId: String,
            sessionId: String,
            messageId: String,
            sequence: Long,
            action: RemoteAction,
            params: Map<String, String> = emptyMap(),
            leaseId: String? = null,
            controlEpoch: Long = 0,
            ttlMs: Long = 250,
            nowMs: Long = System.currentTimeMillis()
        ): CommandEnvelope {
            return CommandEnvelope(
                robotId = robotId,
                sourceId = sourceId,
                sessionId = sessionId,
                messageId = messageId,
                sequence = sequence,
                issuedAtMs = nowMs,
                ttlMs = ttlMs,
                controlEpoch = controlEpoch,
                leaseId = leaseId,
                action = action.wireValue,
                params = params
            )
        }
    }
}

object CommandValidator {
    fun validate(command: CommandEnvelope): List<String> {
        val errors = mutableListOf<String>()
        if (command.schemaVersion != SNAPPY_REMOTE_SCHEMA_VERSION) errors += "schema_version must be 1"
        if (command.kind != PacketKind.COMMAND.wireValue) errors += "kind must be command"
        if (command.robotId.isBlank()) errors += "robot_id is required"
        if (command.linkId != LinkId.G20_UDP.wireValue) errors += "link_id is invalid"
        if (command.sourceId.isBlank()) errors += "source_id is required"
        if (command.sessionId.isBlank()) errors += "session_id is required"
        if (command.messageId.isBlank()) errors += "message_id is required"
        if (command.sequence < 0) errors += "sequence is invalid"
        if (command.issuedAtMs < 0) errors += "issued_at_ms is invalid"
        if (command.ttlMs !in 1..2000) errors += "ttl_ms must be within 1..2000"
        if (command.action.isBlank()) errors += "action is required"
        return errors
    }
}