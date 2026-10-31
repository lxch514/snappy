"""SQLite collector core with an MQTT-agnostic message boundary."""

from __future__ import annotations

import argparse
import json
import sqlite3
import sys
import time
from pathlib import Path
from typing import Any


MAX_MESSAGE_BYTES = 64 * 1024
UPSTREAM_PREFIX = "snappy/v1/robots/"
ALLOWED_UPSTREAM_SUFFIXES = {
    "/up/ack",
    "/up/state",
    "/up/telemetry",
    "/up/event",
    "/up/presence",
}


class CollectorStore:
    def __init__(self, database: str | Path = ":memory:") -> None:
        self.connection = sqlite3.connect(str(database))
        self.connection.row_factory = sqlite3.Row
        schema = Path(__file__).resolve().parents[1] / "schema.sql"
        self.connection.executescript(schema.read_text(encoding="utf-8"))

    def close(self) -> None:
        self.connection.close()

    def ingest(
        self,
        topic: str,
        payload: str | bytes | dict[str, Any],
        received_at_ms: int | None = None,
    ) -> int:
        if not topic.startswith(UPSTREAM_PREFIX) or not any(
            topic.endswith(suffix) for suffix in ALLOWED_UPSTREAM_SUFFIXES
        ):
            raise ValueError(f"unsupported upstream topic: {topic}")

        if isinstance(payload, dict):
            raw_payload = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
        elif isinstance(payload, bytes):
            raw_payload = payload.decode("utf-8")
        else:
            raw_payload = payload
        if len(raw_payload.encode("utf-8")) > MAX_MESSAGE_BYTES:
            raise ValueError("message exceeds 64 KiB")

        value = json.loads(raw_payload)
        if not isinstance(value, dict):
            raise ValueError("message must be a JSON object")
        if value.get("kind") == "video" or "video" in topic.lower():
            raise ValueError("video is outside the cloud data boundary")
        robot_id = value.get("robot_id")
        kind = value.get("kind")
        if not isinstance(robot_id, str) or not robot_id:
            raise ValueError("robot_id is required")
        if not isinstance(kind, str) or not kind:
            raise ValueError("kind is required")

        received = received_at_ms if received_at_ms is not None else int(time.time() * 1000)
        cursor = self.connection.execute(
            """
            INSERT INTO messages(robot_id, topic, kind, received_at_ms, payload)
            VALUES (?, ?, ?, ?, ?)
            """,
            (robot_id, topic, kind, received, raw_payload),
        )
        if topic.endswith("/up/state"):
            self.connection.execute(
                """
                INSERT INTO latest_state(robot_id, received_at_ms, payload)
                VALUES (?, ?, ?)
                ON CONFLICT(robot_id) DO UPDATE SET
                  received_at_ms = excluded.received_at_ms,
                  payload = excluded.payload
                """,
                (robot_id, received, raw_payload),
            )
        self.connection.commit()
        return int(cursor.lastrowid)

    def latest_state(self, robot_id: str) -> dict[str, Any] | None:
        row = self.connection.execute(
            "SELECT payload FROM latest_state WHERE robot_id = ?", (robot_id,)
        ).fetchone()
        return None if row is None else json.loads(row["payload"])

    def prune_before(self, cutoff_ms: int) -> int:
        cursor = self.connection.execute(
            "DELETE FROM messages WHERE received_at_ms < ?", (cutoff_ms,)
        )
        self.connection.commit()
        return cursor.rowcount


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Collect newline-delimited fake/MQTT-adapter messages into SQLite"
    )
    parser.add_argument("--database", default="snappy-cloud.db")
    args = parser.parse_args(argv)

    store = CollectorStore(args.database)
    try:
        for line in sys.stdin:
            envelope = json.loads(line)
            store.ingest(envelope["topic"], envelope["payload"])
    finally:
        store.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
