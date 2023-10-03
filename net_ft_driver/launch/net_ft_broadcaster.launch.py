# Copyright (c) 2022, Grzegorz Bartyzel
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import launch
import launch_ros
from launch.actions import OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ip_address = LaunchConfiguration("ip_address")
    rdt_sampling_rate = LaunchConfiguration("rdt_sampling_rate")
    sensor_type = LaunchConfiguration("sensor_type")
    internal_filter_rate = LaunchConfiguration("internal_filter_rate")
    use_hardware_biasing = LaunchConfiguration("use_hardware_biasing")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("net_ft_description"),
                    "urdf",
                    "net_ft_sensor.urdf.xacro",
                ]
            ),
            " ",
            "ip_address:=",
            ip_address,
            " ",
            "rdt_sampling_rate:=",
            rdt_sampling_rate,
            " ",
            "sensor_type:=",
            sensor_type,
            " ",
            "internal_filter_rate:=",
            internal_filter_rate,
            " ",
            "use_hardware_biasing:=",
            use_hardware_biasing,
            " ",
        ]
    )
    robot_description_param = {"robot_description": robot_description_content}

    ft_controller = PathJoinSubstitution(
        [FindPackageShare("net_ft_driver"), "config", "net_ft_broadcaster.yaml"]
    )

    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description_param, ft_controller],
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_param],
    )

    force_torque_sensor_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "force_torque_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    net_ft_diagnostic_broadcaster = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "net_ft_diagnostic_broadcaster",
            "-c",
            "/controller_manager",
        ],
    )

    nodes = [
        control_node,
        robot_state_publisher_node,
        force_torque_sensor_broadcaster_spawner,
        net_ft_diagnostic_broadcaster,
    ]

    return nodes


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="sensor_type",
            default_value="ati_axia",
            description="Type of the F/T sensor.",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="ip_address",
            default_value="192.168.1.10",
            description="F/T Sensor IP adress",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="rdt_sampling_rate",
            default_value="500",
            description="The RDT sampling rate.",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="internal_filter_rate",
            default_value="false",
            description=(
                "The internal low pass filter rate, "
                "refer for specific values to the sensor manuals.",
            ),
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="use_hardware_biasing",
            default_value="0",
            description="Whether to use built-in sensor zeroing",
        )
    )

    return launch.LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
