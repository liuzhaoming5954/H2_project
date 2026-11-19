"""Regression checks for the map bounds that destabilized static replay."""

import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "lio_replay_config", ROOT / "src/h2_bringup/launch/lio_replay_config.py")
VALIDATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATION)


class LioReplayConfiguration(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="h2-lio-replay-")
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "replay.yaml"
        self.config = yaml.safe_load(
            (ROOT / "src/h2_bringup/config/fastlio_static_replay.yaml").read_text())
        self.parameters = self.config["/**"]["ros__parameters"]

    def write(self):
        self.path.write_text(yaml.safe_dump(self.config))
        return str(self.path)

    def test_checked_in_configuration_has_sufficient_map_margin(self):
        parameters = VALIDATION.load_lio_replay_config(self.write(), True)
        self.assertEqual(parameters["cube_side_length"], 1000.0)
        self.assertEqual(parameters["mapping.det_range"], 100.0)

    def test_oscillating_map_dimensions_and_boundary_are_rejected(self):
        for cube in (200.0, 300.0):
            with self.subTest(cube=cube):
                self.parameters["cube_side_length"] = cube
                with self.assertRaisesRegex(VALIDATION.LioReplayConfigError, "must exceed"):
                    VALIDATION.load_lio_replay_config(self.write(), True)

    def test_nonfinite_and_nonpositive_dimensions_are_rejected(self):
        for value in (0, -1, float("nan"), float("inf"), True):
            with self.subTest(value=value):
                self.parameters["mapping.det_range"] = value
                with self.assertRaises(VALIDATION.LioReplayConfigError):
                    VALIDATION.load_lio_replay_config(self.write(), True)

    def test_uncalibrated_extrinsics_require_explicit_opt_in(self):
        with self.assertRaisesRegex(VALIDATION.LioReplayConfigError, "allow_experimental"):
            VALIDATION.load_lio_replay_config(self.write())

    def test_mid360s_manual_nominal_config_uses_inverse_translation_in_meters(self):
        config_path = ROOT / "src/h2_bringup/config/fastlio_mid360s_nominal.yaml"
        parameters = VALIDATION.load_lio_replay_config(str(config_path))
        self.assertEqual(parameters["mapping.extrinsic_T"], [-0.011, -0.02329, 0.04412])
        self.assertEqual(parameters["mapping.extrinsic_R"],
                         [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertFalse(parameters["mapping.extrinsic_est_en"])

    def test_unknown_translation_is_rejected_even_with_experimental_opt_in(self):
        for translation in ([0.011, 0.02329, -0.04412], [-11.0, -23.29, 44.12],
                            [0.01, 0.0, 0.0], [False, 0.0, 0.0], [float("nan"), 0.0, 0.0]):
            with self.subTest(translation=translation):
                self.parameters["mapping.extrinsic_T"] = translation
                with self.assertRaises(VALIDATION.LioReplayConfigError):
                    VALIDATION.load_lio_replay_config(self.write(), True)

    def test_extrinsic_estimation_and_input_topics_remain_fixed(self):
        self.parameters["mapping.extrinsic_est_en"] = True
        with self.assertRaises(VALIDATION.LioReplayConfigError):
            VALIDATION.load_lio_replay_config(self.write(), True)
        self.parameters["mapping.extrinsic_est_en"] = False
        self.parameters["common.imu_topic"] = "/livox/imu"
        with self.assertRaises(VALIDATION.LioReplayConfigError):
            VALIDATION.load_lio_replay_config(self.write(), True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
