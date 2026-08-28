from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    autostart_param = DeclareLaunchArgument(
        name='autostart',
        default_value='True',
        description='Automatically start lifecycle nodes')

    kinova_controller_runner = Node(
        package='kinova_controller',
        executable='run',
        output='screen',
        arguments=[
            '--autostart', LaunchConfiguration('autostart'),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(autostart_param)
    ld.add_action(kinova_controller_runner)

    return ld
