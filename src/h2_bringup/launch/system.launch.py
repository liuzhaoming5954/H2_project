from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_mode(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("sim", "replay", "hardware_readonly", "hardware"):
        raise RuntimeError(f"Unsupported mode: {mode}")
    domain_text = LaunchConfiguration("ros_domain_id").perform(context)
    domain_text = domain_text or ("81" if mode == "hardware_readonly" else "71")
    if not domain_text.isdecimal() or not 0 <= int(domain_text) <= 232:
        raise RuntimeError("ros_domain_id must be an integer from 0 to 232")
    isolation = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", domain_text),
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
        SetEnvironmentVariable("ROS_STATIC_PEERS", ""),
    ]
    if mode == "sim":
        config = str(Path(get_package_share_directory("h2_bringup")) / "config/sim.yaml")
        return isolation + [
            LogInfo(msg="Mock motion only. Explicit enable and fresh permits/commands are required."),
            Node(package="h2_runtime", executable="motion_guard", output="screen", parameters=[config]),
            Node(package="h2_runtime", executable="mock_platform", output="screen", parameters=[config]),
            Node(package="h2_supervisor", executable="supervisor", output="screen", parameters=[config]),
        ]
    if mode == "replay":
        return isolation + [LogInfo(msg=(
            "Replay isolation: no motion writer or mock TF publisher is launched. "
            "Play an explicitly selected bag separately; LIO/map algorithms are not integrated yet."))]
    if mode == "hardware_readonly":
        sensors_launch = str(Path(get_package_share_directory("h2_bringup")) /
                             "launch/sensors_readonly.launch.py")
        return isolation + [
            LogInfo(msg=("First hardware test: MID-360 acquisition only. "
                         "H2 state collection uses the separate passive SDK probe.")),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch),
                launch_arguments={
                    "lidar_config": LaunchConfiguration("lidar_config"),
                    "ros_domain_id": domain_text,
                }.items()),
        ]
    if mode == "hardware":
        raise RuntimeError(
            "H2 motion SDK bridge and hardware health policy are not implemented. "
            "Use hardware_readonly for first sensor tests; motion launch is rejected.")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="sim"),
        DeclareLaunchArgument("ros_domain_id", default_value=""),
        DeclareLaunchArgument("lidar_config", default_value=""),
        OpaqueFunction(function=launch_mode),
    ])
