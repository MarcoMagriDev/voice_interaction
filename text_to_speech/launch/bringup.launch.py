#
# Copyright (c) 2024 Marco Magri
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    mission_manager_node = Node(
        package="text_to_speech",
        executable="text_to_speech_node.py",
        output="both",
        parameters=[LaunchConfiguration("config_file")],
        # arguments=["--ros-args", "--log-level", log_level],
    )

    return [mission_manager_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=f"{get_package_share_directory('text_to_speech')}/config/default.yaml",
                description="Absolute path of the configuration file",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
