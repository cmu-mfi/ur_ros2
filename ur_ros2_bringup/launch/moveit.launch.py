from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os
import yaml
import xacro
import tempfile

def load_file(package_name, file_path):
    """Load a plain text file (like SRDF) into a string."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    """Load a YAML file into a Python dictionary."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def launch_setup(context, *args, **kwargs):
    ns = context.launch_configurations['ns']
    model = context.launch_configurations['model']
    log_level = context.launch_configurations['log_level']
    tf_prefix = LaunchConfiguration("tf_prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

    if ns == '':
        prefix = ''
        namespace = ''
    else:
        prefix = ns + "_"
        namespace = '/' + ns

    print("")
    print("Starting moveit with parameters:")
    print(" log_level:           " + log_level)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + ns)
    print(" model:               " + model)
    print("")

    # Package names (Update these to match your workspace)
    config_package = 'ur_ros2_bringup'

    # SRDF
    srdf_file_path = os.path.join(get_package_share_directory(config_package), 'srdf', 'ur.srdf.xacro')
    srdf_config = xacro.process_file(srdf_file_path, mappings={"name": model, "tf_prefix": prefix})
    srdf = {'robot_description_semantic': srdf_config.toxml()}

    # Kinematics
    kinematics_template_path = os.path.join(get_package_share_directory(config_package), 'config', 'kinematics.yaml')
    with open(kinematics_template_path, 'r') as f:
        content = f.read()
        modified_content = content.replace("<prefix>", prefix)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as temp_file:
        temp_file.write(modified_content)
        kinematics_path = temp_file.name
    with open(kinematics_path, 'r') as file:
        kinematics_yaml = yaml.safe_load(file)
    kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Joint Limits (Planning constraints)
    joint_limits_template_path = os.path.join(get_package_share_directory(config_package), 'config', 'joint_limits.yaml')
    with open(joint_limits_template_path, 'r') as f:
        content = f.read()
        modified_content = content.replace("<prefix>", prefix)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as temp_file:
        temp_file.write(modified_content)
        joint_limits_path = temp_file.name
    with open(joint_limits_path, 'r') as file:
        joint_limits_yaml = yaml.safe_load(file)
    joint_limits = {'robot_description_planning': joint_limits_yaml}

    # Planning Pipeline
    pilz_yaml = load_yaml(config_package, 'config/pilz_industrial_motion_planner_planning.yaml')
    pilz_planning_pipeline_config = {
        'default_planning_pipeline': 'pilz_industrial_motion_planner',
        'planning_pipelines': ['pilz_industrial_motion_planner'],
        'pilz_industrial_motion_planner': pilz_yaml  # This provides the necessary nesting!
    }
    cartesian_limits_yaml = load_yaml(config_package, 'config/pilz_cartesian_limits.yaml')
    cartesian_limits = {'robot_description_planning': cartesian_limits_yaml}

    # 6. Trajectory Execution and Controllers
    moveit_controllers_template_path = os.path.join(get_package_share_directory(config_package), 'config', 'moveit_controllers.yaml')
    with open(moveit_controllers_template_path, 'r') as f:
        content = f.read()
        modified_content = content.replace("<prefix>", prefix)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as temp_file:
        temp_file.write(modified_content)
        moveit_controllers_path = temp_file.name
    with open(moveit_controllers_path, 'r') as file:
        moveit_controllers_yaml = yaml.safe_load(file)
    moveit_controllers = moveit_controllers_yaml

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }
    move_group_capabilities = {
        "disable_capabilities": "".join([
            "move_group/ClearOctomapService",
        ]) 
    }
    disable_occupancy_map = {
        "sensors": [""],          # Do not load any 3D sensor plugins
        "octomap_frame": "",    # Clear the reference frame for the octomap
        "octomap_resolution": 0.0,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=ns,
        output='screen',
        parameters=[
            srdf,
            kinematics,
            joint_limits,
            cartesian_limits,
            pilz_planning_pipeline_config,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            disable_occupancy_map,
            {'use_sim_time': False} 
        ],
        arguments=[
            '--ros-args', 
            '--log-level', 
            'test.moveit.moveit.ros.occupancy_map_monitor:=FATAL',
            '--log-level', 
            log_level
        ]
    )

    # Servo
    servo_parameters_template_path = os.path.join(get_package_share_directory(config_package), 'config', 'servo_parameters.yaml')
    with open(servo_parameters_template_path, 'r') as f:
        content = f.read()
        modified_content = content.replace("<prefix>", prefix).replace("<namespace>", namespace)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as temp_file:
        temp_file.write(modified_content)
        servo_parameters_path = temp_file.name
    with open(servo_parameters_path, 'r') as file:
        servo_parameters_yaml = yaml.safe_load(file)
    servo_params = {"moveit_servo": servo_parameters_yaml}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        condition=IfCondition(launch_servo),
        namespace=ns,
        parameters=[
            srdf,
            kinematics,
            joint_limits,
            cartesian_limits,
            pilz_planning_pipeline_config,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            disable_occupancy_map,
            servo_params,
        ],
        output="screen",
        arguments=[
            '--ros-args', 
            '--log-level', 
            log_level
        ]
    )

    # Rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_ros2_bringup"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        namespace=ns,
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            srdf,
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
                'ns',
                default_value='',
                description='namespace of the robot (used as prefix, so needed if running multiple robots)'
                )
            )
    declared_arguments.append(
            SetLaunchConfiguration('tf_prefix', PythonExpression(["'", LaunchConfiguration('ns'), "' + '_' if '", LaunchConfiguration('ns'), "' else ''"]))
    )
    declared_arguments.append(
            SetLaunchConfiguration('ur_type', "ur20")
    )
    declared_arguments.append(
            DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?"),
            )
    declared_arguments.append(
            DeclareLaunchArgument("launch_servo", default_value="false", description="Launch Moveit Servo?"),
            )
    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            default_value='ur20',
            description="Type/series of used UR robot.",
            choices=[
                "ur3", "ur5", "ur10", "ur3e", "ur5e", "ur7e",
                "ur10e", "ur12e", "ur16e", "ur8long", "ur15",
                "ur18", "ur20", "ur30",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            default_value='ur20',
            description="Type/series of used UR robot.",
            choices=[
                "ur3", "ur5", "ur10", "ur3e", "ur5e", "ur7e",
                "ur10e", "ur12e", "ur16e", "ur8long", "ur15",
                "ur18", "ur20", "ur30",
            ],
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
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
