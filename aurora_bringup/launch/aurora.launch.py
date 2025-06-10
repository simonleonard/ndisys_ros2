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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="aurora_description",
            description="Description package with robot URDF/XACRO files."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="aurora_bringup",
            description="Package with the controller\'s configuration in config folder."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="aurora.urdf.xacro",
            description="URDF/XACRO description file with the aurora.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="aurora_broadcaster.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "com_port",
            default_value='/dev/ttyUSB0',
            description="COM port"
        )
    )

    # General arguments
    description_package = LaunchConfiguration("description_package")
    runtime_config_package = LaunchConfiguration("runtime_config_package")

    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")
    
    com_port = LaunchConfiguration("com_port")
    
    aurora_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "name:=jhu",
            " ",
            "com_port:=",
            com_port,
            " ",
        ]
    )
    aurora_description = {"robot_description": aurora_description_content}

    aurora_controller_config = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    aurora_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        emulate_tty=True,
        parameters=[aurora_description, aurora_controller_config],
        output={"screen"},
    )

    aurora_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            # long timeout because NDI takes a while to configure
            "--controller-manager-timeout",
            "30"
        ] + ["ndi"],
        remappings=[
            ("/rigid_poses", "/poses")    
        ]
    )
    
    nodes_to_start = [
        aurora_control_node,
        aurora_broadcaster,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
