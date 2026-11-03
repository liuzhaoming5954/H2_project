"""Offline checks for fail-closed first-test configuration validation."""

from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "mid360_config", ROOT / "src/h2_bringup/launch/mid360_config.py")
VALIDATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATION)


class Mid360Configuration(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="h2-mid360-config-")
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "mid360.json"
        self.config = json.loads((ROOT / "src/h2_bringup/config/mid360.example.json").read_text())
        for name in ("cmd_data", "push_msg", "point_data", "imu_data"):
            self.config["MID360"]["host_net_info"][f"{name}_ip"] = "192.168.31.10"
        self.config["lidar_configs"][0]["ip"] = "192.168.31.20"

    def write(self, config=None):
        self.path.write_text(json.dumps(self.config if config is None else config))
        return str(self.path)

    def assert_rejected(self, config):
        with self.assertRaises(VALIDATION.Mid360ConfigError):
            VALIDATION.load_mid360_config(self.write(config))

    def test_valid_single_raw_sensor_configuration(self):
        result = VALIDATION.load_mid360_config(self.write())
        self.assertEqual(result, {
            "path": str(self.path), "host_ip": "192.168.31.10", "lidar_ip": "192.168.31.20"})

    def test_example_placeholders_cannot_start_a_driver(self):
        with self.assertRaises(VALIDATION.Mid360ConfigError):
            VALIDATION.load_mid360_config(str(ROOT / "src/h2_bringup/config/mid360.example.json"))

    def test_path_must_be_explicit_absolute_and_readable(self):
        for value in ("", "mid360.json", str(self.path)):
            with self.subTest(value=value), self.assertRaises(VALIDATION.Mid360ConfigError):
                VALIDATION.load_mid360_config(value)
        with self.assertRaises(VALIDATION.Mid360ConfigError):
            VALIDATION.load_mid360_config(str(self.path.parent))

    def test_duplicate_json_keys_and_nonfinite_constants_are_rejected(self):
        for content in ('{"MID360": {}, "MID360": {}}', '{"value": NaN}', '{"value": Infinity}'):
            self.path.write_text(content)
            with self.subTest(content=content), self.assertRaises(VALIDATION.Mid360ConfigError):
                VALIDATION.load_mid360_config(str(self.path))

    def test_multiple_or_mixed_devices_are_rejected(self):
        config = deepcopy(self.config)
        config["lidar_configs"].append(deepcopy(config["lidar_configs"][0]))
        self.assert_rejected(config)
        config = deepcopy(self.config)
        config["HAP"] = {}
        self.assert_rejected(config)
        config = deepcopy(self.config)
        config["lidar_summary_info"]["lidar_type"] = 9
        self.assert_rejected(config)

    def test_invalid_and_conflicting_network_addresses_are_rejected(self):
        for address in ("::1", "0.0.0.0", "127.0.0.1", "239.1.1.1", "255.255.255.255", 1234):
            config = deepcopy(self.config)
            config["lidar_configs"][0]["ip"] = address
            with self.subTest(address=address):
                self.assert_rejected(config)
        config = deepcopy(self.config)
        config["lidar_configs"][0]["ip"] = "192.168.31.10"
        self.assert_rejected(config)
        config = deepcopy(self.config)
        config["MID360"]["host_net_info"]["imu_data_ip"] = "192.168.31.11"
        self.assert_rejected(config)
        config = deepcopy(self.config)
        config["MID360"]["host_net_info"]["log_data_ip"] = "192.168.31.11"
        self.assert_rejected(config)

    def test_port_numbers_are_valid_and_unique(self):
        for group in ("host_net_info", "lidar_net_info"):
            for port in (0, 65536, "56301", True):
                config = deepcopy(self.config)
                config["MID360"][group]["point_data_port"] = port
                with self.subTest(group=group, port=port):
                    self.assert_rejected(config)
            config = deepcopy(self.config)
            net = config["MID360"][group]
            net["point_data_port"] = net["imu_data_port"]
            self.assert_rejected(config)

    def test_raw_geometry_and_packet_format_are_preserved(self):
        for field, value in (("pcl_data_type", 2), ("pattern_mode", 1), ("pcl_data_type", True)):
            config = deepcopy(self.config)
            config["lidar_configs"][0][field] = value
            self.assert_rejected(config)
        for value in (0.1, "0", False):
            config = deepcopy(self.config)
            config["lidar_configs"][0]["extrinsic_parameter"]["yaw"] = value
            self.assert_rejected(config)

    def test_domain_id_is_numeric_and_bounded(self):
        self.assertEqual(VALIDATION.validate_domain_id("81"), "81")
        self.assertEqual(VALIDATION.validate_domain_id("000"), "0")
        for domain in ("-1", "233", "1.0", "81; echo unsafe", "", 81):
            with self.subTest(domain=domain), self.assertRaises(VALIDATION.Mid360ConfigError):
                VALIDATION.validate_domain_id(domain)


if __name__ == "__main__":
    unittest.main(verbosity=2)
