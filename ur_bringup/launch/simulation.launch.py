from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess

def launch_setup(context, *args, **kwargs):
    ip = context.launch_configurations['ip']
    model = context.launch_configurations['model']
    id = int(ip.split('.')[-1])
    port = str(8000 + id)
    node_name = "simulation"

    print("")
    print("Starting simulation with paramaters:")
    print(" ip:           " + ip)
    print(" model:        " + model)
    print(" vnc-port:     " + port)
    print("")
    print("Access the simulation on: http://localhost:" + port + "/vnc.html")
    print("")

    simulation = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ur_client_library', 'start_ursim.sh',
            '-m', model,
            '-n', node_name,
            '-i', ip,
            '-f', '-p ' + port + ':6080'
        ],
        output='screen'
    )
    return [simulation]

def generate_launch_description():
    # add launch arguments
    launch_arguments = []
    launch_arguments.append(
            DeclareLaunchArgument(
                'ip',
                default_value='192.168.20.101',
                description='ip address of robot'
                )
            )
    launch_arguments.append(
            DeclareLaunchArgument(
                'model',
                default_value='ur20',
                description='model of robot [ur20, ur10, ...]'
                )
            )
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])

