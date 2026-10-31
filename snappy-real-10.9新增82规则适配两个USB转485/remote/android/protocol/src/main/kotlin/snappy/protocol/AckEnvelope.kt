/**
 * @file AckEnvelope.kt
 * @brief 应答信封
 *
 * @details
 * - 携带受理状态、执行进度与失败原因。
 */


package snappy.protocol

data class AckEnvelope(
    val schemaVersion: Int = SNAPPY_REMOTE_SCHEMA_VERSION,
    val kind: String = PacketKind.ACK.wireValue,
    val robotId: String,
    val messageId: String,
    val sequence: Long,
    val status: AckStatus,
    val detail: String = "",
    val retryable: Boolean = false,
    val leaseId: String = "",
    val controlEpoch: Long = 0,
    val receivedAtMs: Long = System.currentTimeMillis()
)