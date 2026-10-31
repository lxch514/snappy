import json
import unittest

from cloud.cli.snappy_cloudctl import FakePublisher, build_command, publication_for


class CloudctlTests(unittest.TestCase):
    def test_publication_uses_mqtt_backup_link_and_safe_delivery_flags(self) -> None:
        command = build_command(
            robot_id="snappy-001",
            source_id="operator-01",
            boot_id="boot-0001",
            action="control.acquire",
            now_ms=123,
        )
        publication = publication_for(command)
        publisher = FakePublisher()
        publisher.publish(publication)

        self.assertEqual(command["link_id"], "ec20_mqtt")
        self.assertEqual(publication.qos, 1)
        self.assertFalse(publication.retain)
        self.assertEqual(
            publication.topic, "snappy/v1/robots/snappy-001/down/command"
        )
        self.assertEqual(json.loads(publication.payload)["action"], "control.acquire")
        self.assertEqual(publisher.publications, [publication])

    def test_ttl_cannot_exceed_contract_limit(self) -> None:
        with self.assertRaises(ValueError):
            build_command(
                robot_id="snappy-001",
                source_id="operator-01",
                boot_id="boot-0001",
                action="control.acquire",
                ttl_ms=1001,
            )

    def test_boot_id_must_be_explicit(self) -> None:
        with self.assertRaises(ValueError):
            build_command(
                robot_id="snappy-001",
                source_id="operator-01",
                boot_id="",
                action="control.acquire",
            )


if __name__ == "__main__":
    unittest.main()
