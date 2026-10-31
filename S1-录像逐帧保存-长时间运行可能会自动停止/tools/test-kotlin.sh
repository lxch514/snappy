#!/usr/bin/env bash
set -euo pipefail

SNAPPY_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SNAPPY_BUILD_DIR=${SNAPPY_BUILD_DIR:-"$SNAPPY_ROOT/build"}
KOTLIN_VERSION=1.9.24

if [[ -n "${KOTLINC:-}" ]]; then
  KOTLIN_COMPILER=$KOTLINC
elif command -v kotlinc >/dev/null 2>&1; then
  KOTLIN_COMPILER=$(command -v kotlinc)
else
  KOTLIN_TOOL_DIR="$SNAPPY_BUILD_DIR/toolchains/kotlin-$KOTLIN_VERSION"
  KOTLIN_ARCHIVE="$SNAPPY_BUILD_DIR/toolchains/kotlin-compiler-$KOTLIN_VERSION.zip"
  KOTLIN_COMPILER="$KOTLIN_TOOL_DIR/kotlinc/bin/kotlinc"
  if [[ ! -x "$KOTLIN_COMPILER" ]]; then
    if [[ "${SNAPPY_NO_TOOLCHAIN_DOWNLOAD:-0}" == "1" ]]; then
      echo "kotlinc is missing and toolchain download is disabled" >&2
      exit 1
    fi
    mkdir -p "$KOTLIN_TOOL_DIR" "$(dirname "$KOTLIN_ARCHIVE")"
    if [[ ! -f "$KOTLIN_ARCHIVE" ]]; then
      curl --fail --location --proto '=https' --tlsv1.2 \
        "https://github.com/JetBrains/kotlin/releases/download/v$KOTLIN_VERSION/kotlin-compiler-$KOTLIN_VERSION.zip" \
        --output "$KOTLIN_ARCHIVE"
    fi
    unzip -q -o "$KOTLIN_ARCHIVE" -d "$KOTLIN_TOOL_DIR"
  fi
fi

KOTLIN_OUTPUT_DIR="$SNAPPY_BUILD_DIR/kotlin"
KOTLIN_JAR="$KOTLIN_OUTPUT_DIR/snappy-kotlin-tests.jar"
mkdir -p "$KOTLIN_OUTPUT_DIR"

mapfile -d '' KOTLIN_SOURCES < <(
  find "$SNAPPY_ROOT/remote/android" -type f -name '*.kt' -print0 | sort -z
)
if [[ ${#KOTLIN_SOURCES[@]} -eq 0 ]]; then
  echo "no Kotlin sources found" >&2
  exit 1
fi

"$KOTLIN_COMPILER" \
  "${KOTLIN_SOURCES[@]}" \
  -jvm-target 1.8 \
  -include-runtime \
  -d "$KOTLIN_JAR"

java -cp "$KOTLIN_JAR" snappy.protocol.ContractGoldenTestKt \
  "$SNAPPY_ROOT/contracts/v1/test-vectors"
