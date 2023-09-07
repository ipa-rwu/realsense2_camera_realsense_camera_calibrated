# Copyright (c) 2023 Ruichao Wu
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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

configurable_parameters = [
    {
        "name": "translation_x",
        "description": "translation x",
    },
    {
        "name": "translation_y",
        "description": "translation y",
    },
    {
        "name": "translation_z",
        "description": "translation z",
    },
    {
        "name": "rotation_x",
        "description": "rotation x",
    },
    {
        "name": "rotation_y",
        "description": "rotation y",
    },
    {
        "name": "rotation_z",
        "description": "rotation z",
    },
    {
        "name": "rotation_w",
        "description": "rotation w",
    },
    {
        "name": "child_frame_id",
        "description": "child frame id",
    },
    {
        "name": "frame_id",
        "description": "frame id",
    },
]


def declare_configurable_parameters(parameters: dict):
    return [
        DeclareLaunchArgument(
            param["name"],
            default_value=param["default"],
            description=param["description"],
        )
        if "default" in list(param.keys())
        else DeclareLaunchArgument(
            param["name"],
            description=param["description"],
        )
        for param in parameters
    ]


def set_configurable_parameters(parameters):
    return dict(
        [(param["name"], LaunchConfiguration(param["name"])) for param in parameters]
    )


def launch_setup(context, params):
    realsence_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            LaunchConfiguration("translation_x"),
            LaunchConfiguration("translation_y"),
            LaunchConfiguration("translation_z"),
            LaunchConfiguration("rotation_x"),
            LaunchConfiguration("rotation_y"),
            LaunchConfiguration("rotation_z"),
            LaunchConfiguration("rotation_w"),
            LaunchConfiguration("child_frame_id"),
            LaunchConfiguration("frame_id"),
        ],
    )
    return [realsence_tf_node]


def generate_launch_description():
    realsence_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        )
    )

    entites = declare_configurable_parameters(configurable_parameters)
    entites.append(realsence_launch)
    ld = LaunchDescription(entites)

    ld.add_action(
        OpaqueFunction(
            function=launch_setup,
            kwargs={"params": set_configurable_parameters(configurable_parameters)},
        )
    )

    return ld
