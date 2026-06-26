from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression 
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context):
    # Load parameters
    log_level = context.launch_configurations['log_level']
    ns = context.launch_configurations['ns']
    tf_prefix = context.launch_configurations['tf_prefix']
    model = context.launch_configurations['model']
    launch_servo = context.launch_configurations['launch_servo']
    launch_rviz = context.launch_configurations['launch_rviz']
    kinematics_params_file = context.launch_configurations['kinematics_params_file']

    # print parameters
    print("")
    print("Starting moveit with parameters:")
    print(" log_level:           " + log_level)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + "/" + ns)
    print(" model:               " + model)
    print(" launch_servo:        " + launch_servo)
    print(" launch_rviz:         " + launch_rviz)
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
    joint_limits = {'robot_description_planning': 
                    {
                        "default_velocity_scaling_factor": 1.0,
                        "default_acceleration_scaling_factor": 1.0,
                        "joint_limits": prefixed_limits,
                        }
                    }

    # Planning Pipeline
    pilz_config_path = os.path.join(get_package_share_directory(pkg_prefix+"bringup"), 'config', 'pilz_industrial_motion_planner_planning.yaml')
    with open(pilz_config_path, 'r') as file:
        pilz_config_yaml = yaml.safe_load(file)
    planning_pipeline = {
            "planning_pipelines": {
                'pipeline_names': ["pilz_industrial_motion_planner"]
                },
            "default_planning_pipeline": "pilz_industrial_motion_planner",
            'pilz_industrial_motion_planner': pilz_config_yaml,
            }

    # Cartesian Limits
    cartesian_limits_path = os.path.join(get_package_share_directory(pkg_prefix+"bringup"), 'config', 'pilz_cartesian_limits.yaml')
    with open(cartesian_limits_path, 'r') as file:
        cartesian_limits_yaml = yaml.safe_load(file)
    cartesian_limits = {'robot_description_planning': cartesian_limits_yaml}

    # Trajectory Execution
    moveit_controllers_path = os.path.join(get_package_share_directory(pkg_prefix+"bringup"), 'config', 'moveit_controllers.yaml')
    trajectory_execution = ParameterFile(moveit_controllers_path, allow_substs=True)

    # Planning Scene Parameters
    planning_scene_parameters = {
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "publish_robot_description": False,
            "publish_robot_description_semantic": False,
            }

    # MoveGroup Node
    move_group_node = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            namespace=ns,
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics,
                joint_limits,
                planning_pipeline,
                cartesian_limits,
                trajectory_execution,
                planning_scene_parameters,
                {'use_sim_time': False} 
                ],
            arguments=[
                '--ros-args', 
                '--log-level', 
                ns+'.moveit.moveit.ros.occupancy_map_monitor:=FATAL',
                '--log-level', 
                'moveit.moveit.ros.occupancy_map_monitor:=FATAL',
                '--log-level', 
                log_level
                ],
            sigterm_timeout='1.0',
            sigkill_timeout='1.0'
            )

    # Servo
    servo_parameters_path = os.path.join(get_package_share_directory(pkg_prefix+"bringup"), 'config', 'servo_parameters.yaml')
    servo_parameters = ParameterFile(servo_parameters_path, allow_substs=True)
    servo_node = Node(
            package="moveit_servo",
            executable="servo_node",
            namespace=ns,
            condition=IfCondition(launch_servo),
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics,
                joint_limits,
                servo_parameters,
                ],
            output="screen",
            arguments=[
                '--ros-args', 
                '--log-level', 
                log_level
                ],
            sigterm_timeout='1.0',
            sigkill_timeout='1.0'
            )

    # RVIZ
    rviz_config_file = PathJoinSubstitution(
            [FindPackageShare(pkg_prefix+"bringup"), "config", "moveit.rviz"]
            )
    rviz_node = Node(
            package="rviz2",
            condition=IfCondition(launch_rviz),
            executable="rviz2",
            name="rviz2_moveit",
            namespace=ns,
            output="log",
            arguments=["-d", rviz_config_file, '--ros-args', '--log-level', log_level],
            parameters=[
                robot_description,
                robot_description_semantic,
                planning_scene_parameters,
                kinematics,
                {
                    "use_sim_time": False,
                    },
                ],
            )

    return [move_group_node, servo_node, rviz_node]

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
            DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?"),
            )
    declared_arguments.append(
            DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Moveit Servo?"),
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
