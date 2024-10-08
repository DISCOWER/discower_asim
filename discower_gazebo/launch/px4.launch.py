#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

from utilities.utilities import *


def generate_launch_description():
    """Launch Gazebo with a drone running PX4 communicating over ROS 2."""

    px4_dir = get_package_share_directory("px4")
    return LaunchDescription(
        [
            DeclareLaunchArgument("id", default_value="0"),
            ExecuteProcess(
                cmd=[
                    "gnome-terminal",
                    "--",
                    px4_dir + "/px4_sitl_default/bin/px4",
                    px4_dir + "/px4_sitl_default/etc",
                    "-s",
                    "etc/init.d-posix/rcS",
                    "-i",
                    LaunchConfiguration("id"),
                    "",

                ],
                cwd=px4_dir,
                additional_env={"PX4_SIM_SPEED_FACTOR": "1",
                                "PX4_SIM_MODEL": "gazebo-classic_2d_spacecraft"},
                output="screen",
            ),
        ]
    )
