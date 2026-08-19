"""Bring up the RealSense D435i (aligned color+depth) and the VRPN bridge to
Motive (OptiTrack), so both streams are live for `ros2 bag record`.

Standalone file -- no package/build needed. Run directly with:
    ros2 launch scripts/launch_camera_and_mocap.launch.py server:=<motive_pc_ip>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    server = LaunchConfiguration("server")
    port = LaunchConfiguration("port")

    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )
    vrpn_launch = os.path.join(
        get_package_share_directory("vrpn_mocap"), "launch", "client.launch.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "server", description="Motive PC's IP address on the lab network"
        ),
        DeclareLaunchArgument(
            "port", default_value="3883", description="VRPN streaming port (Motive default: 3883)"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                "enable_color": "true",
                "enable_depth": "true",
                "align_depth.enable": "true",
                "enable_sync": "true",
                "rgb_camera.color_profile": "640x480x30",
                "depth_module.depth_profile": "640x480x30",
            }.items(),
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(vrpn_launch),
            launch_arguments={"server": server, "port": port}.items(),
        ),
    ])
