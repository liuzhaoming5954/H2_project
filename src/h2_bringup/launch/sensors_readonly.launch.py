"""MID-360/MID-360S capture only: no H2 command writer, estimator, TF or mock state."""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validator():
    helper = Path(__file__).with_name("mid360_config.py")
    spec = importlib.util.spec_from_file_location("h2_mid360_readonly_config", helper)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _unexpected_driver_exit(event, context):
    if context.is_shutdown:
        return []
    # Let LaunchService report an error and shut down the remaining actions.
    # A driver that exits immediately must not look like a successful capture.
    raise RuntimeError(f"Livox driver exited unexpectedly with code {event.returncode}")


def start_sensor(context):
    validation = _validator()
    domain = validation.validate_domain_id(LaunchConfiguration("ros_domain_id").perform(context))
    config = validation.load_mid360_config(LaunchConfiguration("lidar_config").perform(context))
    driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="mid360_readonly_driver",
        namespace="/",
        output="screen",
        parameters=[{
            "xfer_format": 1,
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": 10.0,
            "output_data_type": 0,
            "frame_id": "livox_frame",
            "user_config_path": config["path"],
        }],
        remappings=[("livox/lidar", "/livox/lidar"), ("livox/imu", "/livox/imu")],
        respawn=False,
    )
    return [
        SetEnvironmentVariable("ROS_DOMAIN_ID", domain),
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
        SetEnvironmentVariable("ROS_STATIC_PEERS", ""),
        LogInfo(msg=(
            f"Livox sensor capture: host={config['host_ip']}, lidar={config['lidar_ip']}, "
            f"ROS domain={domain}. Publishes raw /livox/lidar and /livox/imu only. "
            "Livox SDK configures sensor output; no H2 command writer is launched. "
            "Process liveness alone does not prove valid sensor data."
        )),
        RegisterEventHandler(OnProcessExit(target_action=driver, on_exit=_unexpected_driver_exit)),
        driver,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_config", default_value="",
            description="Explicit absolute path to the reviewed single MID-360/MID-360S JSON configuration"),
        DeclareLaunchArgument(
            "ros_domain_id", default_value="81",
            description="ROS sensor-capture domain, separate from the default mock domain 71"),
        OpaqueFunction(function=start_sensor),
    ])
