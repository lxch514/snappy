import json
import unittest

from cloud.collector.collector import CollectorStore


class CollectorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.store = CollectorStore()

    def tearDown(self) -> None:
        self.store.close()

    def test_state_is_written_to_messages_and_latest_state(self) -> None:
        state = {
            "schema_version": 1,
            "kind": "state",
            "robot_id": "snappy-001",
            "actual": {"control_state": "DISARMED"},
        }
        row_id = self.store.ingest(
            "snappy/v1/robots/snappy-001/up/state", state, received_at_ms=100
        )
        self.assertGreater(row_id, 0)
        self.assertEqual(self.store.latest_state("snappy-001"), state)

    def test_raw_ack_is_preserved(self) -> None:
        raw = json.dumps(
            {"schema_version": 1, "kind": "ack", "robot_id": "snappy-001"},
            separators=(",", ":"),
        )
        self.store.ingest("snappy/v1/robots/snappy-001/up/ack", raw)
        stored = self.store.connection.execute("SELECT payload FROM messages").fetchone()
        self.assertEqual(stored["payload"], raw)

    def test_video_topic_has_no_cloud_entry(self) -> None:
        with self.assertRaises(ValueError):
            self.store.ingest(
                "snappy/v1/robots/snappy-001/up/video",
                {"kind": "video", "robot_id": "snappy-001"},
            )
        count = self.store.connection.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
        self.assertEqual(count, 0)


if __name__ == "__main__":
    unittest.main()
