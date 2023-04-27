from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sofar_printer_simulator',
            executable='printer_sim_node',
            name='printer_node',
        ),
    ])
