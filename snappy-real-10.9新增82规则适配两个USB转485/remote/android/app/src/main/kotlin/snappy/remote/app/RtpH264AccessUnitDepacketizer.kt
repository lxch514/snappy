/**
 * @file RtpH264AccessUnitDepacketizer.kt
 * @brief RTP/H264 分片重组
 *
 * @details
 * - 处理单包、分片起始、分片中间与分片结束四种载荷类型。
 * - 只在重建出完整访问单元后回调一帧。
 * - 序号不连续时丢弃当前分片，避免拼出损坏帧。
 */
 
package snappy.remote.app

import java.io.ByteArrayOutputStream

class RtpH264AccessUnitDepacketizer(
    private val onFrame: (ByteArray) -> Unit
) {
    private var fuBuffer: ByteArrayOutputStream? = null
    private var lastSeq: Int? = null
    private val accessUnit = ByteArrayOutputStream(256 * 1024)

    fun consume(packet: ByteArray, offset: Int, length: Int) {
        if (length < 13) return

        val base = offset
        val end = offset + length

        val version = (packet[base].toInt() ushr 6) and 0x03
        if (version != 2) return

        val seq = ((packet[base + 2].toInt() and 0xff) shl 8) or
            (packet[base + 3].toInt() and 0xff)

        val marker = (packet[base + 1].toInt() and 0x80) != 0

        val previousSeq = lastSeq
        if (previousSeq != null && ((previousSeq + 1) and 0xffff) != seq) {
            fuBuffer = null
            accessUnit.reset()
        }
        lastSeq = seq

        val csrcCount = packet[base].toInt() and 0x0f
        var payloadOffset = base + 12 + csrcCount * 4

        if (payloadOffset >= end) return

        if ((packet[base].toInt() and 0x10) != 0) {
            if (payloadOffset + 4 > end) return
            val extensionLength =
                ((packet[payloadOffset + 2].toInt() and 0xff) shl 8) or
                    (packet[payloadOffset + 3].toInt() and 0xff)
            payloadOffset += 4 + extensionLength * 4
        }

        if (payloadOffset >= end) return

        when (packet[payloadOffset].toInt() and 0x1f) {
            in 1..23 -> appendNal(packet, payloadOffset, end - payloadOffset)
            24 -> consumeStapA(packet, payloadOffset + 1, end)
            28 -> consumeFuA(packet, payloadOffset, end)
        }

        if (marker) {
            emitAccessUnit()
        }
    }

    private fun consumeStapA(packet: ByteArray, start: Int, end: Int) {
        var offset = start

        while (offset + 2 <= end) {
            val nalSize =
                ((packet[offset].toInt() and 0xff) shl 8) or
                    (packet[offset + 1].toInt() and 0xff)

            offset += 2

            if (nalSize <= 0 || offset + nalSize > end) {
                return
            }

            appendNal(packet, offset, nalSize)
            offset += nalSize
        }
    }

    private fun consumeFuA(packet: ByteArray, offset: Int, end: Int) {
        if (offset + 2 >= end) return

        val indicator = packet[offset].toInt() and 0xff
        val header = packet[offset + 1].toInt() and 0xff

        val start = (header and 0x80) != 0
        val finish = (header and 0x40) != 0
        val nalHeader = ((indicator and 0xe0) or (header and 0x1f)).toByte()

        if (start) {
            fuBuffer = ByteArrayOutputStream(end - offset).apply {
                write(nalHeader.toInt())
            }
        }

        val buffer = fuBuffer ?: return
        buffer.write(packet, offset + 2, end - offset - 2)

        if (finish) {
            val nal = buffer.toByteArray()
            appendNal(nal, 0, nal.size)
            fuBuffer = null
        }
    }

    private fun appendNal(packet: ByteArray, offset: Int, size: Int) {
        accessUnit.write(START_CODE, 0, START_CODE.size)
        accessUnit.write(packet, offset, size)
    }

    private fun emitAccessUnit() {
        if (accessUnit.size() > 0) {
            onFrame(accessUnit.toByteArray())
            accessUnit.reset()
        }
    }

    companion object {
        private val START_CODE = byteArrayOf(0, 0, 0, 1)
    }
}