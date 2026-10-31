import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


class ConfigurationBoundaryTests(unittest.TestCase):
    def test_hardware_schema_declares_canonical_device_ids(self) -> None:
        schema = json.loads(
            (ROOT / "rk3588" / "config" / "hardware.schema.json").read_text(
                encoding="utf-8"
            )
        )

        devices = schema["properties"]["devices"]
        self.assertFalse(devices["additionalProperties"])
        self.assertEqual(
            set(devices["required"]),
            {
                "lidar_brush_left",
                "lidar_brush_right",
                "lidar_body_front",
                "lidar_body_rear",
                "anemometer_left",
                "anemometer_right",
                "wit_motion",
                "temperature_cabinet",
                "power_24v",
                "power_48v",
                "relay_push_motor",
                "brush_motor",
                "climber",
                "propulsion_fan",
            },
        )

        relay_outputs = schema["properties"]["relay_outputs"]
        self.assertFalse(relay_outputs["additionalProperties"])
        self.assertEqual(
            set(relay_outputs["required"]),
            {
                "brush_push_extend",
                "brush_push_retract",
                "scraper_push_extend",
                "scraper_push_retract",
            },
        )

    def test_unknown_real_hardware_parameters_remain_null(self) -> None:
        config = json.loads(
            (ROOT / "rk3588" / "config" / "hardware.example.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(config["template_scope"], "fake_only")
        self.assertEqual(config["can_buses"], {})
        self.assertEqual(config["rs485_ports"], {})
        for device in config["devices"].values():
            self.assertEqual(device["backend"], "fake")
            self.assertIsNone(device["bus_id"])
            self.assertIsNone(device["model"])
            self.assertIsNone(device["protocol_family"])
            self.assertIsNone(device["protocol_profile"])
            if "can_id" in device:
                self.assertIsNone(device["can_id"])
            if "station_id" in device:
                self.assertIsNone(device["station_id"])
        for mapping in config["relay_outputs"].values():
            self.assertEqual(mapping["controller_id"], "relay_controller")
            self.assertIsNone(mapping["physical_channel"])

    def test_link_roles_and_video_boundary_are_fixed(self) -> None:
        config = json.loads(
            (ROOT / "rk3588" / "config" / "hardware.example.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(config["links"]["g20_udp"]["role"], "primary_control")
        self.assertEqual(
            config["links"]["ec20_mqtt"]["role"], "auxiliary_backup"
        )
        for link_id in ("g20_udp", "ec20_mqtt"):
            self.assertIsNone(config["links"][link_id]["heartbeat_ms"])
            self.assertIsNone(config["links"][link_id]["lease_timeout_ms"])
        self.assertEqual(
            config["provisional_fake_values"]["g20_udp"],
            {"heartbeat_ms": 250, "lease_timeout_ms": 750},
        )
        self.assertEqual(
            config["provisional_fake_values"]["ec20_mqtt"],
            {"heartbeat_ms": 1000, "lease_timeout_ms": 3000},
        )
        self.assertNotIn("websocket", config["links"])
        self.assertFalse(config["video"]["cloud_export"])


if __name__ == "__main__":
    unittest.main()