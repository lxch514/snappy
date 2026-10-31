"""Human-operated emergency command builder.

The real MQTT/TLS publisher is intentionally an interface only. The default CLI
prints a QoS-1, non-retained publication for review or fake integration tests.
"""

from __future__ import annotations

import argparse
import json
import time
import uuid
from dataclasses import dataclass
from typing import Any, Protocol


@dataclass(frozen=True)
class Publication:
    topic: str
    payload: str
    qos: int = 1
    retain: bool = False


class Publisher(Protocol):
    def publish(self, publication: Publication) -> None: ...


class FakePublisher:
    def __init__(self) -> None:
        self.publications: list[Publication] = []

    def publish(self, publication: Publication) -> None:
        self.publications.append(publication)


def build_command(
    *,
    robot_id: str,
    source_id: str,
    boot_id: str,
    action: str,
    params: dict[str, Any] | None = None,
    control_epoch: int = 0,
    lease_id: str | None = None,
    sequence: int = 0,
    ttl_ms: int = 1000,
    now_ms: int | None = None,
) -> dict[str, Any]:
    if not 1 <= ttl_ms <= 1000:
        raise ValueError("ttl_ms must be within 1..1000")
    if not boot_id:
        raise ValueError("boot_id must come from the latest device presence/state")
    timestamp = now_ms if now_ms is not None else int(time.time() * 1000)
    return {
        "schema_version": 1,
        "kind": "command",
        "robot_id": robot_id,
        "link_id": "ec20_mqtt",
        "source_id": source_id,
        "boot_id": boot_id,
        "session_id": str(uuid.uuid4()),
        "message_id": str(uuid.uuid4()),
        "seq": sequence,
        "issued_at_ms": timestamp,
        "ttl_ms": ttl_ms,
        "control_epoch": control_epoch,
        "lease_id": lease_id,
        "action": action,
        "params": params or {},
    }


def publication_for(command: dict[str, Any]) -> Publication:
    robot_id = command["robot_id"]
    return Publication(
        topic=f"snappy/v1/robots/{robot_id}/down/command",
        payload=json.dumps(command, separators=(",", ":"), ensure_ascii=False),
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build a human-operated MQTT command")
    parser.add_argument("robot_id")
    parser.add_argument("action", help="use control.acquire before emergency commands")
    parser.add_argument("--source-id", default="snappy-cloudctl")
    parser.add_argument(
        "--boot-id",
        required=True,
        help="current device boot_id from presence/state; stale IDs are rejected by the device",
    )
    parser.add_argument("--params", default="{}", help="JSON object")
    parser.add_argument("--control-epoch", type=int, default=0)
    parser.add_argument("--lease-id")
    parser.add_argument("--ttl-ms", type=int, default=1000)
    args = parser.parse_args(argv)

    params = json.loads(args.params)
    if not isinstance(params, dict):
        parser.error("--params must be a JSON object")
    command = build_command(
        robot_id=args.robot_id,
        source_id=args.source_id,
        boot_id=args.boot_id,
        action=args.action,
        params=params,
        control_epoch=args.control_epoch,
        lease_id=args.lease_id,
        ttl_ms=args.ttl_ms,
    )
    publication = publication_for(command)
    print(
        json.dumps(
            {
                "topic": publication.topic,
                "qos": publication.qos,
                "retain": publication.retain,
                "payload": json.loads(publication.payload),
                "mode": "dry-run; real MQTT/TLS publisher not configured",
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
