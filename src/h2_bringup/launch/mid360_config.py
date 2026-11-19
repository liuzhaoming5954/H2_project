"""Validate the intentionally narrow, single MID-360 family first-test configuration.

This module uses only the Python standard library and never opens a network
socket. Validation proves configuration shape, not sensor connectivity or time
synchronization. The Livox driver/SDK will configure the sensor's output stream.
"""

import ipaddress
import json
import math
from pathlib import Path
import re


MAX_CONFIG_BYTES = 64 * 1024
PORT_NAMES = (
    "cmd_data", "push_msg", "point_data", "imu_data", "log_data",
)


class Mid360ConfigError(ValueError):
    """The supplied file is unsuitable for the first read-only sensor test."""


def _object(value, label, expected_keys):
    if not isinstance(value, dict) or set(value) != set(expected_keys):
        raise Mid360ConfigError(f"{label}: expected fields {', '.join(expected_keys)}")
    return value


def _unique_json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise Mid360ConfigError(f"Duplicate JSON field: {key}")
        result[key] = value
    return result


def _reject_json_constant(value):
    raise Mid360ConfigError(f"Non-finite JSON constant is not allowed: {value}")


def _ipv4(value, label):
    if not isinstance(value, str):
        raise Mid360ConfigError(f"{label}: an explicit IPv4 address is required")
    try:
        address = ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError as error:
        raise Mid360ConfigError(f"{label}: replace placeholders with a valid IPv4 address") from error
    if address.is_unspecified or address.is_loopback or address.is_multicast or address.is_reserved:
        raise Mid360ConfigError(f"{label}: a unicast sensor-network IPv4 address is required")
    return str(address)


def _ports(mapping, label):
    values = []
    for name in PORT_NAMES:
        port = mapping[f"{name}_port"]
        if type(port) is not int or not 1 <= port <= 65535:
            raise Mid360ConfigError(f"{label}.{name}_port: expected an integer in 1..65535")
        values.append(port)
    if len(set(values)) != len(values):
        raise Mid360ConfigError(f"{label}: port numbers must be distinct")


def validate_domain_id(value):
    """Return a canonical ROS domain string without changing the environment."""
    if not isinstance(value, str) or not re.fullmatch(r"[0-9]{1,3}", value):
        raise Mid360ConfigError("ros_domain_id must be an integer in 0..232")
    number = int(value)
    if number > 232:
        raise Mid360ConfigError("ros_domain_id must be an integer in 0..232")
    return str(number)


def load_mid360_config(config_path):
    """Validate an explicit absolute file and return its path/address summary.

    Raw sensor geometry is retained: only Livox Cartesian data, its normal
    non-repetitive pattern and zero SDK extrinsics are accepted. Mounting and
    LiDAR/IMU transforms belong to the later project calibration/TF adapter.
    """
    if not isinstance(config_path, str) or not config_path:
        raise Mid360ConfigError("lidar_config must name an explicit absolute MID-360 JSON file")
    path = Path(config_path)
    if not path.is_absolute():
        raise Mid360ConfigError("lidar_config must be an absolute path")
    try:
        path = path.resolve(strict=True)
        if not path.is_file() or path.stat().st_size > MAX_CONFIG_BYTES:
            raise Mid360ConfigError("lidar_config must be a regular JSON file no larger than 64 KiB")
        config = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_unique_json_object,
            parse_constant=_reject_json_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise Mid360ConfigError(f"Cannot read MID-360 JSON configuration: {error}") from error

    model_keys = {"MID360", "Mid360s"}.intersection(config) if isinstance(config, dict) else set()
    if len(model_keys) != 1:
        raise Mid360ConfigError("Configuration requires exactly one device key: MID360 or Mid360s")
    model_key = model_keys.pop()
    _object(config, "configuration", ("lidar_summary_info", model_key, "lidar_configs"))
    summary = _object(config["lidar_summary_info"], "lidar_summary_info", ("lidar_type",))
    if type(summary["lidar_type"]) is not int or summary["lidar_type"] != 8:
        raise Mid360ConfigError("lidar_summary_info.lidar_type must be 8 (Livox protocol)")
    model = _object(config[model_key], model_key, ("lidar_net_info", "host_net_info"))
    port_fields = tuple(f"{name}_port" for name in PORT_NAMES)
    lidar_label = f"{model_key}.lidar_net_info"
    lidar_net = _object(model["lidar_net_info"], lidar_label, port_fields)
    _ports(lidar_net, lidar_label)
    host_label = f"{model_key}.host_net_info"
    if model_key == "Mid360s":
        hosts = model["host_net_info"]
        if not isinstance(hosts, list) or len(hosts) != 1:
            raise Mid360ConfigError(f"{host_label}: expected a single PC3 host entry")
        host_label += "[0]"
        host_net = _object(hosts[0], host_label, ("host_ip", *port_fields))
        host_ip = _ipv4(host_net["host_ip"], f"{host_label}.host_ip")
    else:
        host_net = _object(model["host_net_info"], host_label,
                           tuple(field for name in PORT_NAMES
                                 for field in (f"{name}_ip", f"{name}_port")))
        host_addresses = [
            _ipv4(host_net[f"{name}_ip"], f"{host_label}.{name}_ip")
            for name in PORT_NAMES if name != "log_data"
        ]
        if len(set(host_addresses)) != 1:
            raise Mid360ConfigError("All host stream addresses must select the same PC3 IPv4 address")
        host_ip = host_addresses[0]
        log_ip = host_net["log_data_ip"]
        if log_ip != "" and _ipv4(log_ip, f"{host_label}.log_data_ip") != host_ip:
            raise Mid360ConfigError("log_data_ip must be empty or equal to the PC3 host address")
    _ports(host_net, host_label)

    devices = config["lidar_configs"]
    if not isinstance(devices, list) or len(devices) != 1:
        raise Mid360ConfigError("First sensor test requires exactly one sensor in lidar_configs")
    device = _object(devices[0], "lidar_configs[0]",
                     ("ip", "pcl_data_type", "pattern_mode", "extrinsic_parameter"))
    lidar_ip = _ipv4(device["ip"], "lidar_configs[0].ip")
    if lidar_ip == host_ip:
        raise Mid360ConfigError("LiDAR and PC3 host IPv4 addresses must differ")
    if type(device["pcl_data_type"]) is not int or device["pcl_data_type"] != 1:
        raise Mid360ConfigError("pcl_data_type must be 1 for the selected raw Cartesian format")
    if type(device["pattern_mode"]) is not int or device["pattern_mode"] != 0:
        raise Mid360ConfigError("pattern_mode must remain 0 for the first sensor test")
    extrinsics = _object(device["extrinsic_parameter"], "extrinsic_parameter",
                         ("roll", "pitch", "yaw", "x", "y", "z"))
    for name, value in extrinsics.items():
        if type(value) not in (int, float) or not math.isfinite(value) or value != 0:
            raise Mid360ConfigError(f"extrinsic_parameter.{name} must remain zero for raw sensor data")
    return {"path": str(path), "host_ip": host_ip, "lidar_ip": lidar_ip}
