#pragma once
/**
 * @file result.hpp
 * @brief 驱动层错误码与统一返回类型
 *
 * @details
 * - Result 同时携带错误码、说明、发生阶段和设备名。
 * - 调用方据此区分可重试、需配置和致命错误。
 */

#include <string>
#include <utility>

namespace snappy::drivers {

enum class ErrorCode {
  kOk,
  kConfigRequired,
  kInvalidArgument,
  kUnsupported,

  kCanTxTimeout,
  kCanFrameInvalid,

  kRs485Timeout,
  kRs485FrameIncomplete,
  kRs485CrcMismatch,
  kRs485AddressMismatch,
  kRs485FunctionMismatch,
  kRs485LengthMismatch,
  kRs485ExceptionResponse,

  kRelayInterlockActive,
  kSensorStale,
  kSafeHoldActive,
  kIoError,
};

struct Result {
  bool ok{false};
  ErrorCode code{ErrorCode::kIoError};
  std::string component;
  std::string operation;
  bool retryable{false};
  std::string detail;

  static Result success(std::string detail = {}) {
    return {
        true,
        ErrorCode::kOk,
        {},
        {},
        false,
        std::move(detail)};
  }

  static Result failure(
      ErrorCode code,
      std::string detail,
      std::string operation = {},
      std::string component = {},
      bool retryable = false) {
    return {
        false,
        code,
        std::move(component),
        std::move(operation),
        retryable,
        std::move(detail)};
  }

  static Result failure(
      std::string detail) {
    return failure(
        ErrorCode::kIoError,
        std::move(detail));
  }
};

[[nodiscard]] const char* error_code_name(
    ErrorCode code) noexcept;

}  // namespace snappy::drivers