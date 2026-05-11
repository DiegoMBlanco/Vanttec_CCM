from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'f1tenth_simulation',
            executable = 'MPC_accel_heading.py',
            output = 'screen'),
        Node(
            package = 'f1tenth_simulation',
            executable = 'path_drawer.py',
            output = 'screen'),
    ])
