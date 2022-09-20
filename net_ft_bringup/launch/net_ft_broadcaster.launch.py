import launch
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
import launch_ros
import os


def generate_launch_description():
    description_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="net_ft_description"
    ).find("net_ft_description")
    default_model_path = os.path.join(
        description_pkg_share, "urdf", "ati_axia80_ft_sensor.urdf.xacro"
    )

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="net_ft_driver_bringup"
    ).find("net_ft_driver_bringup")

    args = []
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to gripper URDF file",
        )
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("model"),
            " ",
            "ip_address:=192.168.1.1",
        ]
    )
    robot_description_param = {
        "robot_description": launch_ros.parameter_descriptions.ParameterValue(
            robot_description_content, value_type=str
        )
    }

    controllers_file = "ati_axia80_broadcaster.yaml"
    ft_controller = PathJoinSubstitution(
        [pkg_share, "config", controllers_file]
    )


    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_param,
            ft_controller,
        ],
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

    """
    net_ft_diagnostic_broadcaster = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "net_ft_diagnostic_broadcaster", "-c", "/controller_manager",
        ],
    )
    """

    nodes = [
        control_node,
        robot_state_publisher_node,
        force_torque_sensor_broadcaster_spawner,
        # net_ft_diagnostic_broadcaster,
    ]

    return launch.LaunchDescription(args + nodes)
