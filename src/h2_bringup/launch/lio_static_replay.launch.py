"""Start FAST-LIO for a local static bag experiment with fixed experimental extrinsics."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_replay(context):
    helper = Path(__file__).with_name("lio_replay_config.py")
    spec = importlib.util.spec_from_file_location("h2_lio_replay_config", helper)
    validation = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validation)
    params_file = LaunchConfiguration("params_file").perform(context)
    allowed = LaunchConfiguration("allow_experimental_extrinsics").perform(context) == "true"
    parameters = validation.load_lio_replay_config(params_file, allowed)
    namespace = validation.REPLAY_NAMESPACE
    return [
        SetEnvironmentVariable("ROS_DOMAIN_ID", "167"),
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
        SetEnvironmentVariable("ROS_STATIC_PEERS", ""),
        LogInfo(msg=(
            "FAST-LIO static replay uses ROS domain 167 and experimental identity extrinsics. "
            "Outputs retain native camera_init/body frames."
        )),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_static",
            namespace=namespace,
            output="screen",
            parameters=[parameters],
            remappings=[
                ("/Odometry", namespace + "/odometry"),
                ("/cloud_registered", namespace + "/world_cloud"),
                ("/cloud_registered_body", namespace + "/body_cloud"),
                ("/cloud_effected", namespace + "/effects"),
                ("/Laser_map", namespace + "/map"),
                ("/path", namespace + "/path"),
                ("/tf", namespace + "/tf"),
                ("/tf_static", namespace + "/tf_static"),
            ],
            respawn=False,
        ),
    ]


def generate_launch_description():
    config = Path(get_package_share_directory("h2_bringup")) / "config/fastlio_static_replay.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=str(config)),
        DeclareLaunchArgument("allow_experimental_extrinsics", default_value="false"),
        OpaqueFunction(function=start_replay),
    ])
