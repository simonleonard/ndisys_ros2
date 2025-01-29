# Copyright 2023 ICube Laboratory, University of Strasbourg
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
# Author: Adnan SAOOD

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os

description_package = "aurora_description"
description_file = "aurora.config.xacro"
aurora_runtime_config_package = "aurora_bringup"
controllers_file = "aurora_broadcaster.yaml"


def launch_setup(context, *args, **kwargs):
    urdf_path = os.path.join(
        get_package_share_directory('aurora_description'),
        'config',
        'aurora.config.xacro')

    desc_file = xacro.parse(open(urdf_path))
    xacro.process_doc(desc_file)
    aurora_description = {"robot_description": desc_file.toxml()}

    aurora_controller_config = os.path.join(
        get_package_share_directory("aurora_bringup"),
        "config",
        "aurora_broadcaster.yaml")

    aurora_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        emulate_tty=True,
        parameters=[aurora_description, aurora_controller_config],
        output={"screen"},
    )

    aurora_controller = Node(
           package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                # long timeout because NDI takes a while to configure
                "--controller-manager-timeout",
                "30"
            ] + ["rigid_pose_broadcaster"],
    )
    
    nodes_to_start = [
        aurora_control_node,
        aurora_controller
    ]

    return nodes_to_start


def generate_launch_description():

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
