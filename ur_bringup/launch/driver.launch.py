from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition

def launch_setup(context):
    ns = context.launch_configurations['ns']
    tf_prefix = context.launch_configurations['tf_prefix']
    ip = context.launch_configurations['ip']
    use_mock_hardware = context.launch_configurations['use_mock_hardware']
    log_level = context.launch_configurations['log_level']
    model = context.launch_configurations['model']
    kinematics_params_file = context.launch_configurations['kinematics_params_file']

    print("")
    print("Starting bringup with paramaters:")
    print(" log_level:           " + log_level)
    if use_mock_hardware == "false":
        print(" ip:                  " + ip)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + ns)
    print(" use_mock_hardware:   " + use_mock_hardware)
    print(" model:               " + model)
    print("")

    # Packages
    config_pkg_name = "ur_bringup"
    description_pkg_name = "ur_robot_driver"

    # ROS2 Controllers
    ros2_controllers_file = PathJoinSubstitution(
        [FindPackageShare(config_pkg_name), "config", "ros2_controllers.yaml"]
    )
    # Robot Description
    description_file = PathJoinSubstitution([FindPackageShare(description_pkg_name), "urdf", "ur.urdf.xacro"])
    script_filename = PathJoinSubstitution([FindPackageShare("ur_client_library"), "resources", "external_control.urscript"])
    input_recipe_filename = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"])
    output_recipe_filename = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "robot_ip:=",
            ip,
            " ",
            "ur_type:=",
            model,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "kinematics_params:=",
            kinematics_params_file,
            " ",
            "name:=",
            model,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "headless_mode:=true",
            " ",
            "safety_limits:=true",
            " ",
            "script_filename:=", script_filename,
            " ",
            "input_recipe_filename:=", input_recipe_filename,
            " ",
            "output_recipe_filename:=", output_recipe_filename,
        ]
    )
    robot_description = {
            "robot_description": ParameterValue(robot_description_content, value_type=str)
            }

    nodes = []

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        parameters=[robot_description],
        arguments=["--ros-args", "--log-level", log_level],
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
        "joint_trajectory_controller",
    ]
    # controllers_inactive = [
    #     "passthrough_trajectory_controller",
    #     "force_mode_controller",
    # ]

    nodes.append(controller_spawner(controllers_active, active=True))
    # nodes.append(controller_spawner(controllers_inactive, active=False))

    return nodes


def generate_launch_description():
    declared_arguments = []
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
                "ip", 
                default_value="192.168.19.101",
                description="IP address by which the robot can be reached."
                )
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
    # Parameters declared for ur launch files to work (you dont have to set these)
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

