import json
import unittest
from pathlib import Path

from cloud.cli.snappy_cloudctl import build_command
from contracts.v1 import validate, validate_manifest


ROOT = Path(__file__).resolve().parents[2]
CONTRACT_ROOT = ROOT / "contracts" / "v1"


class ContractTests(unittest.TestCase):
    def test_manifest_vectors_match_expectations(self) -> None:
        results = validate_manifest(CONTRACT_ROOT)
        self.assertEqual(len(results), 4)

    def test_all_schema_files_are_valid_json_objects(self) -> None:
        for path in sorted((CONTRACT_ROOT / "schemas").glob("*.schema.json")):
            with self.subTest(path=path.name):
                schema = json.loads(path.read_text(encoding="utf-8"))
                self.assertEqual(schema["$schema"], "https://json-schema.org/draft/2020-12/schema")
                self.assertEqual(schema["type"], "object")

    def test_cloudctl_command_matches_command_schema(self) -> None:
        command = build_command(
            robot_id="snappy-001",
            source_id="operator-01",
            boot_id="boot-0001",
            action="control.acquire",
            now_ms=1234,
        )
        schema = json.loads(
            (CONTRACT_ROOT / "schemas" / "command.schema.json").read_text(
                encoding="utf-8"
            )
        )
        validate(command, schema)


if __name__ == "__main__":
    unittest.main()
