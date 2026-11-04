"""Validate the local FAST-LIO static replay configuration without starting ROS."""

import math
from pathlib import Path

import yaml


REPLAY_NAMESPACE = "/h2_lio_replay"


class LioReplayConfigError(ValueError):
    """The configuration does not meet the static replay contract."""


def load_lio_replay_config(config_path, allow_experimental_extrinsics=False):
    """Read a replay-only configuration and reject invalid map dimensions."""
    if allow_experimental_extrinsics is not True:
        raise LioReplayConfigError(
            "Set allow_experimental_extrinsics:=true to use the uncalibrated identity extrinsics")
    path = Path(config_path)
    if not path.is_absolute():
        raise LioReplayConfigError("params_file must be an absolute path")
    try:
        if not path.is_file() or path.stat().st_size > 64 * 1024:
            raise LioReplayConfigError("params_file must be a regular YAML file no larger than 64 KiB")
        config = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise LioReplayConfigError(f"Cannot read replay configuration: {error}") from error
    if not isinstance(config, dict) or set(config) != {"/**"}:
        raise LioReplayConfigError("Expected a single /** parameter block")
    node = config["/**"]
    if not isinstance(node, dict) or set(node) != {"ros__parameters"}:
        raise LioReplayConfigError("Expected ros__parameters in the /** block")
    parameters = node["ros__parameters"]
    if not isinstance(parameters, dict):
        raise LioReplayConfigError("ros__parameters must be a mapping")

    cube = parameters.get("cube_side_length")
    detection_range = parameters.get("mapping.det_range")
    for name, value in (("cube_side_length", cube), ("mapping.det_range", detection_range)):
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise LioReplayConfigError(f"{name} must be finite and positive")
    # The pinned FAST-LIO moves its local map at MOV_THRESHOLD * DET_RANGE,
    # where MOV_THRESHOLD is 1.5. Both map edges need that margin.
    if cube <= 3.0 * detection_range:
        raise LioReplayConfigError("cube_side_length must exceed 3 * mapping.det_range")

    required = {
        "use_sim_time": False,
        "common.lid_topic": REPLAY_NAMESPACE + "/raw_lidar",
        "common.imu_topic": REPLAY_NAMESPACE + "/raw_imu",
        "common.time_sync_en": False,
        "common.time_offset_lidar_to_imu": 0.0,
        "preprocess.lidar_type": 1,
        "preprocess.scan_line": 4,
        "mapping.extrinsic_est_en": False,
        "mapping.extrinsic_T": [0.0, 0.0, 0.0],
        "mapping.extrinsic_R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "runtime_pos_log_enable": False,
        "pcd_save.pcd_save_en": False,
    }
    for name, expected in required.items():
        value = parameters.get(name)
        if value != expected or type(value) is not type(expected):
            raise LioReplayConfigError(f"{name} must remain {expected!r} for this replay entry")
    return parameters
