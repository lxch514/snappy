#pragma once

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