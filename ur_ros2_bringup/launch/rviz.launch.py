from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import logging

def launch_setup(context, *args, **kwargs):
    ns = context.launch_configurations['ns']
    model = context.launch_configurations['model']
    launch_rviz = context.launch_configurations['launch_rviz']
    log_level = context.launch_configurations['log_level']

    if ns == '':
        prefix = ''
    else:
        prefix = ns + "_"

    print("")
    print("Starting moveit with parameters:")
    print(" log_level:           " + log_level)
    print(" ns:                  " + ns)
    print(" model:               " + model)
    print(" launch_rviz:         " + launch_rviz)
    print("")
    if log_level == "info":
        logging.getLogger().setLevel(logging.INFO)
    elif log_level == "debug":
        logging.getLogger().setLevel(logging.DEBUG)
    else:
        logging.getLogger().setLevel(logging.ERROR)

    moveit_config = (
        MoveItConfigsBuilder(robot_name=model, package_name="ur_ros2_bringup") 
        .robot_description_semantic(
            file_path="srdf/ur.srdf.xacro",
            mappings={"name": model, "tf_prefix": prefix}
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_ros2_bringup"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        namespace=ns,
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", log_level],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": False,
            },
        ],
    )

    return [rviz_node]


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(
        DeclareLaunchArgument(
            'ns',
            default_value='',
            description='namespace of the robot (used as prefix, so needed if running multiple robots)'
        )
    )
    launch_arguments.append(
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
    launch_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false", description="RViz?")
    )
    launch_arguments.append(
            DeclareLaunchArgument(
                'log_level',
                default_value='error',
                description="Log Level to use for all nodes",
                choices=["info", "debug", "error"],
                )
            )
    
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
