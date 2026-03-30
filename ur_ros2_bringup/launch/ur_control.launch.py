from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    OpaqueFunction,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    # Initialize Arguments
    model = LaunchConfiguration("model")
    ip = LaunchConfiguration("ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    log_level = LaunchConfiguration("log_level")
    tf_prefix = LaunchConfiguration("tf_prefix")
    ns = context.launch_configurations['ns']


    if ns != "":
        controllers_file = PathJoinSubstitution([FindPackageShare("ur_ros2_bringup"), "config", "ns_ros2_controllers.yaml"])
    else:
        controllers_file = PathJoinSubstitution([FindPackageShare("ur_ros2_bringup"), "config", "ros2_controllers.yaml"])

    update_rate_config_file = [PathJoinSubstitution(
            [FindPackageShare("ur_robot_driver"), "config"]), "/", model, "_update_rate.yaml"]

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            update_rate_config_file,
            ParameterFile(controllers_file, allow_substs=True),
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=UnlessCondition(use_mock_hardware),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": ip}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        parameters=[
            {"headless_mode": True},
            {"robot_ip": ip},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": ip}],
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        arguments=["--ros-args", "--log-level", log_level],
    )

    def controller_spawner(controllers):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "controller_manager",
                "--controller-manager-timeout",
                "10",
            ]
            + controllers
            + ["--ros-args", "--log-level", log_level],
        )

    controllers = [
            "joint_state_broadcaster",
            "io_and_status_controller",
            "joint_trajectory_controller",
            "force_torque_sensor_broadcaster",
            "ur_configuration_controller",
            ]

    controller_spawners = [
        controller_spawner(controllers),
    ]

    description_launchfile = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_rsp.launch.py"])

    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile),
        launch_arguments={
            "robot_ip": ip,
            "ur_type": model,
            "tf_prefix": tf_prefix
        }.items(),
    )

    nodes_to_start = [
        control_node,
        robot_state_helper_node,
        rsp,
        dashboard_client_node,
        urscript_interface
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "model",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
        )
    )
    declared_arguments.append(
            DeclareLaunchArgument(
                'ns',
                default_value='',
                description='namespace of the robot (used as prefix, so needed if running multiple robots)'
                )
            )
    declared_arguments.append(
            SetLaunchConfiguration('tf_prefix', PythonExpression(["'", LaunchConfiguration('ns'), "' + '_' if '", LaunchConfiguration('ns'), "' else ''"]))
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ip", description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
            DeclareLaunchArgument(
                'log_level',
                default_value='error',
                description="Log Level to use for all nodes",
                choices=["info", "debug", "error"],
                )
            )
    declared_arguments.append(
            DeclareLaunchArgument(
                'kinematics_params_file',
                default_value=PathJoinSubstitution([FindPackageShare("ur_ros2_bringup"), "config", "calibration.yaml"]),
                )
            )
    declared_arguments.append(
            SetLaunchConfiguration('headless_mode', "true")
            )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port that will be opened to forward URScript commands to the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Port that will be opened for trajectory control.",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
