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
            default_value="192.168.1.1",
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

    return launch.LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
