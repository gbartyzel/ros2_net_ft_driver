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
        description_pkg_share, "urdf", "net_ft_sensor.urdf.xacro"
    )

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="net_ft_driver"
    ).find("net_ft_driver")

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
            "use_sim:=false",
        ]
    )
    robot_description_param = {
        "robot_description": launch_ros.parameter_descriptions.ParameterValue(
            robot_description_content, value_type=str
        )
    }

    update_rate_config_file = PathJoinSubstitution(
        [
            pkg_share,
            "config",
            "ati_axia80_update_rate.yaml",
        ]
    )

    controllers_file = "ati_axia80_broadcaster.yaml"
    initial_joint_controllers = PathJoinSubstitution(
        [pkg_share, "config", controllers_file]
    )

    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_param,
            update_rate_config_file,
            initial_joint_controllers,
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

    net_ft_diagnostic_broadcaster = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "net_ft_diagnostic_broadcaster",
            "-ccontroller-manager",
            "/controller_manager",
        ],
    )

    nodes = [
        control_node,
        robot_state_publisher_node,
        force_torque_sensor_broadcaster_spawner,
        net_ft_diagnostic_broadcaster,
    ]

    return launch.LaunchDescription(args + nodes)
