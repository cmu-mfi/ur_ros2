from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression 

def launch_setup(context):
    # Load parameters
    log_level           = str(context.launch_configurations['log_level']).lower()
    ns                  = str(context.launch_configurations['ns'])
    tf_prefix           = str(context.launch_configurations['tf_prefix'])
    model               = str(context.launch_configurations['model'])
    ip                  = str(context.launch_configurations['ip'])
    use_mock_hardware   = str(context.launch_configurations["use_mock_hardware"]).lower()
    launch_rviz         = str(context.launch_configurations["launch_rviz"]).lower()
    launch_servo        = str(context.launch_configurations["launch_servo"]).lower()

    # print parameters
    print("")
    print("Starting bringup with paramaters:")
    print(" log_level:           " + log_level)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + "/" + ns)
    print(" model:               " + model)
    print(" use_mock_hardware:   " + use_mock_hardware)
    if use_mock_hardware == "false":
        print(" ip:                  " + ip)
    print(" launch_servo:        " + launch_servo)
    print(" launch_rviz:         " + launch_rviz)
    print("")

    # prefix for packages
    pkg_prefix = "ur_"

    # launch files
    driver_launch_path = PathJoinSubstitution([FindPackageShare(pkg_prefix+'bringup'), 'launch', 'driver.launch.py'])
    moveit_launch_path = PathJoinSubstitution([FindPackageShare(pkg_prefix+'bringup'), 'launch', 'moveit.launch.py'])
    robot_manager_launch_path = PathJoinSubstitution([FindPackageShare(pkg_prefix+'bringup'), 'launch', 'robot_manager.launch.py'])

    driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch_path),
            launch_arguments={
                'log_level': log_level,
                'ns': ns,
                'model': model,
                'ip': ip,
                'use_mock_hardware': use_mock_hardware,
                }.items()
            )

    moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch_path),
            launch_arguments={
                'log_level': log_level,
                'ns': ns,
                'model': model,
                'launch_servo': launch_servo,
                'launch_rviz': launch_rviz,
                }.items()
            )

    robot_manager = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_manager_launch_path),
            launch_arguments={
                'log_level': log_level,
                'ns': ns,
                'model': model,
                }.items()
            )

    return [driver, moveit, robot_manager]

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
                'ip',
                default_value='192.168.19.101',
                description='ip address of robot'
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
            DeclareLaunchArgument("launch_servo", default_value="false", description="Launch Moveit Servo?"),
            )
    declared_arguments.append(
            DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?"),
            )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
