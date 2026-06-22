from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression 
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition

def launch_setup(context):
    # Load parameters
    log_level = context.launch_configurations['log_level']
    ns = context.launch_configurations['ns']
    tf_prefix = context.launch_configurations['tf_prefix']
    model = context.launch_configurations['model']
    ip = context.launch_configurations['ip']
    use_mock_hardware = context.launch_configurations['use_mock_hardware']
    kinematics_params_file = context.launch_configurations['kinematics_params_file']

    # print parameters
    print("")
    print("Starting driver with paramaters:")
    print(" log_level:           " + log_level)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + "/" + ns)
    print(" model:               " + model)
    print(" use_mock_hardware:   " + use_mock_hardware)
    if use_mock_hardware == "false":
        print(" ip:                  " + ip)
    print("")

    # prefix for packages
    pkg_prefix = "ur_"

    # robot description
    urdf_file_path = PathJoinSubstitution(
            [FindPackageShare(pkg_prefix+"bringup"), "urdf", "ur.urdf.xacro"]
            )
    urdf_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                urdf_file_path,
                " ",
                "robot_ip:=",
                ip,
                " ",
                "tf_prefix:=",
                tf_prefix,
                " ",
                "kinematics_params:=",
                kinematics_params_file,
                " ",
                "model:=",
                model,
                " ",
                "use_mock_hardware:=",
                use_mock_hardware,
                " ",
                "mock_sensor_commands:=",
                "true",
                ]
            )
    robot_description = {
            "robot_description": ParameterValue(urdf_content, value_type=str)
            }

    # ros2_control config
    ros2_controllers_file = PathJoinSubstitution(
            [FindPackageShare(pkg_prefix+"bringup"), "config", "ros2_controllers.yaml"]
            )

    # nodes
    nodes = []

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            robot_description,
            {'publish_frequency': 500.0}
            ]
        ))

    nodes.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[
            ParameterFile(ros2_controllers_file, allow_substs=True),
            robot_description
            ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        ))

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
        "admittance_controller",
    ]
    controllers_inactive = [
        # "forward_position_controller",
        # "joint_trajectory_controller",
    ]

    nodes.append(controller_spawner(controllers_active, True))
    # nodes.append(controller_spawner(controllers_inactive, False))

    return nodes

def generate_launch_description():
    declared_arguments = []
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
                'model',
                default_value='ur20',
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
                "ip", 
                default_value="192.168.19.101",
                description="IP address by which the robot can be reached."
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
            SetLaunchConfiguration('kinematics_params_file', 
                                   PythonExpression(["'", 
                                                     PathJoinSubstitution([FindPackageShare("ur_description"), "config", LaunchConfiguration("model"), "default_kinematics.yaml"]), 
                                                     "' if '", 
                                                     LaunchConfiguration('use_mock_hardware'), 
                                                     "' == 'true' else '",
                                                     PathJoinSubstitution([FindPackageShare("ur_bringup"), "config", "calibration.yaml"]),
                                                     "'"
                                                     ])))
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
