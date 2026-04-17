from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    ip = context.launch_configurations['ip']
    ns = context.launch_configurations['ns']
    tf_prefix = context.launch_configurations['tf_prefix']
    model = context.launch_configurations['model']
    use_mock_hardware = context.launch_configurations["use_mock_hardware"]
    log_level = context.launch_configurations["log_level"]

    print("")
    print("Starting driver with paramaters:")
    print(" log_level:           " + log_level)
    if use_mock_hardware == "false":
        print(" ip:                  " + ip)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + ns)
    print(" model:               " + model)
    print(" use_mock_hardware:   " + use_mock_hardware)
    print("")

    if ns != "":
        controllers_file = PathJoinSubstitution([FindPackageShare("ur_ros2_bringup"), "config", "ns_ros2_controllers.yaml"])
    else:
        controllers_file = PathJoinSubstitution([FindPackageShare("ur_ros2_bringup"), "config", "ros2_controllers.yaml"])

    nodes = []

    nodes.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[
            ParameterFile(controllers_file, allow_substs=True),
            ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        ))

    nodes.append(GroupAction(
        actions=[
            PushRosNamespace(ns),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_rsp.launch.py"])),
                launch_arguments={
                    "robot_ip": ip,
                    "ur_type": model,
                    "tf_prefix": tf_prefix
                }.items())]
                )
        )

    nodes.append(Node(
        package="ur_robot_driver",
        condition=UnlessCondition(use_mock_hardware),
        namespace=ns,
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": ip}],
        arguments=["--ros-args", "--log-level", log_level],
    ))

    nodes.append(Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        namespace=ns,
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        parameters=[
            {"headless_mode": True},
            {"robot_ip": ip},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    ))

    nodes.append(Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": ip}],
        output="screen",
        namespace=ns,
        condition=UnlessCondition(use_mock_hardware),
        arguments=["--ros-args", "--log-level", log_level],
    ))

    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace=ns,
            arguments=[
                "--controller-manager-timeout",
                "10",
            ]
            + inactive_flags
            + controllers
            + ["--ros-args", "--log-level", log_level],
        )

    controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "force_torque_sensor_broadcaster",
        "ur_configuration_controller",
        "joint_trajectory_controller",
    ]
    controllers_inactive = [
        "passthrough_trajectory_controller",
        "force_mode_controller",
    ]

    nodes.append(controller_spawner(controllers_active, active=True))
    nodes.append(controller_spawner(controllers_inactive, active=False))

    return nodes

def generate_launch_description():
    # add launch arguments
    declared_arguments = []
    declared_arguments.append(
            DeclareLaunchArgument(
                'ip',
                default_value='192.168.19.101',
                description='ip address of robot'
                )
            )
    declared_arguments.append(
            DeclareLaunchArgument(
                'model',
                default_value='ur20',
                description="Typo/series of used UR robot.",
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
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(SetLaunchConfiguration('tf_prefix', PythonExpression(["'", LaunchConfiguration('ns'), "' + '_' if '", LaunchConfiguration('ns'), "' else ''"])))
    declared_arguments.append(
            SetLaunchConfiguration('kinematics_params_file', 
                                   PythonExpression(["'",
                                                     PathJoinSubstitution([FindPackageShare("ur_description"), "config", LaunchConfiguration("model"), "default_kinematics.yaml"]), 
                                                     "' if '", 
                                                     LaunchConfiguration('use_mock_hardware'), 
                                                     "' == 'true' else '",
                                                     PathJoinSubstitution([FindPackageShare("ur_ros2_bringup"), "config", "calibration.yaml"]),
                                                     "'"
                                                     ])))
    declared_arguments.append(
            DeclareLaunchArgument(
                'log_level',
                default_value='error',
                description="Log Level to use for all nodes",
                choices=["info", "debug", "error"],
                )
            )
    declared_arguments.append(SetLaunchConfiguration('headless_mode', "true"))
    declared_arguments.append(SetLaunchConfiguration("reverse_port","50001"))
    declared_arguments.append(SetLaunchConfiguration("script_sender_port","50002"))
    declared_arguments.append(SetLaunchConfiguration("trajectory_port","50003"))
    declared_arguments.append(SetLaunchConfiguration("script_command_port","50004"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

