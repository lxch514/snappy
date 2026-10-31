#include "snappy/drivers/result.hpp"

/**
 * @file result.cpp
 * @brief 驱动统一错误码字符串实现
 */

namespace snappy::drivers {

const char* error_code_name(
    ErrorCode code) noexcept {
  switch (code) {
    case ErrorCode::kOk:
      return "OK";

    case ErrorCode::kConfigRequired:
      return "CONFIG_REQUIRED";

    case ErrorCode::kInvalidArgument:
      return "INVALID_ARGUMENT";

    case ErrorCode::kUnsupported:
      return "UNSUPPORTED";

    case ErrorCode::kCanTxTimeout:
      return "CAN_TX_TIMEOUT";

    case ErrorCode::kCanFrameInvalid:
      return "CAN_FRAME_INVALID";

    case ErrorCode::kRs485Timeout:
      return "RS485_TIMEOUT";

    case ErrorCode::kRs485FrameIncomplete:
      return "RS485_FRAME_INCOMPLETE";

    case ErrorCode::kRs485CrcMismatch:
      return "RS485_CRC_MISMATCH";

    case ErrorCode::kRs485AddressMismatch:
      return "RS485_ADDRESS_MISMATCH";

    case ErrorCode::kRs485FunctionMismatch:
      return "RS485_FUNCTION_MISMATCH";

    case ErrorCode::kRs485LengthMismatch:
      return "RS485_LENGTH_MISMATCH";

    case ErrorCode::kRs485ExceptionResponse:
      return "RS485_EXCEPTION_RESPONSE";

    case ErrorCode::kRelayInterlockActive:
      return "RELAY_INTERLOCK_ACTIVE";

    case ErrorCode::kSensorStale:
      return "SENSOR_STALE";

    case ErrorCode::kSafeHoldActive:
      return "SAFE_HOLD_ACTIVE";

    case ErrorCode::kIoError:
      return "IO_ERROR";
  }

  return "IO_ERROR";
}

}  // namespace snappy::drivers