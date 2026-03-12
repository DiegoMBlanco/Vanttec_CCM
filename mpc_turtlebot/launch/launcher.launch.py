from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'mpc_turtlebot',
            executable = 'MPC_OSQP_executable',
            output = 'screen'),
        Node(
            package = 'mpc_turtlebot',
            executable = 'path_drawer_executable',
            output = 'screen')
    ])
