from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression 
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context):
    # Load parameters
    log_level = context.launch_configurations['log_level']
    ns = context.launch_configurations['ns']
    tf_prefix = context.launch_configurations['tf_prefix']
    model = context.launch_configurations['model']
    kinematics_params_file = context.launch_configurations['kinematics_params_file']

    # print parameters
    print("")
    print("Starting robot_manager with parameters:")
    print(" log_level:           " + log_level)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + "/" + ns)
    print(" model:               " + model)
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
                "model:=",
                model,
                " ",
                "tf_prefix:=",
                tf_prefix,
                " ",
                "kinematics_params:=",
                kinematics_params_file,
                ])
    robot_description = {
            "robot_description": ParameterValue(urdf_content, value_type=str)
            }

    # robot description semantic
    srdf_file_path = PathJoinSubstitution(
            [FindPackageShare(pkg_prefix+"bringup"), "srdf", "ur.srdf.xacro"]
            )
    srdf_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                srdf_file_path,
                " ",
                "model:=",
                model,
                " ",
                "tf_prefix:=",
                tf_prefix,
                ]
            )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(srdf_content, value_type=str)}

    # Kinematics
    kinematics_path = os.path.join(get_package_share_directory(pkg_prefix+"bringup"), 'config', 'kinematics.yaml')
    with open(kinematics_path, 'r') as file:
        kinematics_yaml = yaml.safe_load(file)
    kinematics = {'robot_description_kinematics': {f"{tf_prefix}manipulator": kinematics_yaml}}

    # Joint Limits (Planning constraints)
    joint_limits_path = os.path.join(get_package_share_directory(pkg_prefix+"bringup"), 'config', 'joint_limits.yaml')
    with open(joint_limits_path, 'r') as file:
        joint_limits_yaml = yaml.safe_load(file)
    raw_limits = joint_limits_yaml.get('joint_limits', {})
    prefixed_limits = {
            f"{tf_prefix}{joint_name}": limits 
            for joint_name, limits in raw_limits.items()
            }
    joint_limits = {'robot_description_planning': {"joint_limits": prefixed_limits}}

    # Planning Scene Parameters
    planning_scene_parameters = {
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "publish_robot_description": False,
            "publish_robot_description_semantic": False,
            }

    # Robot Manager
    ee_state_publisher = Node(
        package='ur_robot_manager',
        executable='ee_state_publisher',
        namespace=ns,
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            joint_limits,
            planning_scene_parameters,
            {
                'ns': ns,
                'tf_prefix': tf_prefix,
            },
        ],
        arguments=[
            '--ros-args', 
            '--log-level', 
            log_level
        ]
    )

    return [ee_state_publisher]

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
            SetLaunchConfiguration('kinematics_params_file', 
                                   PythonExpression(["'", 
                                                     PathJoinSubstitution([FindPackageShare("ur_description"), "config", LaunchConfiguration("model"), "default_kinematics.yaml"]), 
                                                     "'"
                                                     ])))
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
