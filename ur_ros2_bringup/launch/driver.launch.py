from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import tempfile

def launch_setup(context, *args, **kwargs):
    ip = context.launch_configurations['ip']
    ns = context.launch_configurations['ns']
    model = context.launch_configurations['model']
    use_mock_hardware = context.launch_configurations["use_mock_hardware"]
    log_level = context.launch_configurations["log_level"]
    id = int(ip.split('.')[-1])
    reverse_port = str(50001 + id)
    script_sender_port = str(50002 + id)
    trajectory_port = str(50003 + id)
    script_command_port = str(50004 + id)

    print("")
    print("Starting driver with paramaters:")
    print(" log_level:           " + log_level)
    if use_mock_hardware == "false":
        print(" ip:                  " + ip)
    print(" ns:                  " + ns)
    print(" model:               " + model)
    print(" use_mock_hardware:   " + use_mock_hardware)
    if use_mock_hardware == "false":
        print(" reverse_port:        " + reverse_port)
        print(" script_sender_port:  " + script_sender_port)
        print(" trajectory_port:     " + trajectory_port)
        print(" script_command_port: " + script_command_port)
    print("")

    ur_control_launch_path = PathJoinSubstitution([FindPackageShare('ur_ros2_bringup'), 'launch', 'ur_control.launch.py'])

    if use_mock_hardware == "true":
        calibration_file = PathJoinSubstitution([FindPackageShare("ur_description"), "config", model, "default_kinematics.yaml"])
    else:
        calibration_file = PathJoinSubstitution([FindPackageShare("ur_ros2_bringup"), "config", "calibration.yaml"])

    ur_control = GroupAction(
        actions=[
            PushRosNamespace(ns),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_control_launch_path),
                launch_arguments={
                    'model': model,
                    'ip': ip,
                    'ns': ns,
                    'use_mock_hardware': use_mock_hardware,
                    'reverse_port': reverse_port,
                    'kinematics_params_file': calibration_file,
                    'script_sender_port': script_sender_port,
                    'trajectory_port': trajectory_port,
                    'script_command_port': script_command_port,
                    'log_level': log_level,
                }.items()
            ),
        ]
    )

    return [ur_control]

def generate_launch_description():
    # add launch arguments
    launch_arguments = []
    launch_arguments.append(
            DeclareLaunchArgument(
                'ip',
                default_value='192.168.19.101',
                description='ip address of robot'
                )
            )
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
    launch_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
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

