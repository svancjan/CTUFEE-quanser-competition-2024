from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ctu_quanser_comp',
            executable='simulationInterface',
            output='screen'
        ),
        Node(
            package='ctu_quanser_comp',
            executable='objectDetection',
            output='screen'
        ),
        Node(
            package='ctu_quanser_comp',
            executable='lateralPlanning',
            output='screen'
        )
    ])
