from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_mode(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode == "sim":
        config = str(Path(get_package_share_directory("h2_bringup")) / "config/sim.yaml")
        return [
            LogInfo(msg="Mock motion only. Explicit enable and fresh permits/commands are required."),
            Node(package="h2_runtime", executable="motion_guard", output="screen", parameters=[config]),
            Node(package="h2_runtime", executable="mock_platform", output="screen", parameters=[config]),
            Node(package="h2_supervisor", executable="supervisor", output="screen", parameters=[config]),
        ]
    if mode == "replay":
        return [LogInfo(msg=(
            "Replay isolation: no motion writer or mock TF publisher is launched. "
            "Play an explicitly selected bag separately; LIO/map algorithms are not integrated yet."))]
    if mode in ("hardware", "hardware_readonly"):
        raise RuntimeError(
            "H2 SDK/sensor bridge and hardware health policy are not implemented in this increment. "
            "Hardware launch is rejected; no simulated state may authorize real motion.")
    raise RuntimeError(f"Unsupported mode: {mode}")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="sim"),
        DeclareLaunchArgument("ros_domain_id", default_value="71"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
        OpaqueFunction(function=launch_mode),
    ])
