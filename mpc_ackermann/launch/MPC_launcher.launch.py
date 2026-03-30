from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'mpc_ackermann',
            executable = 'MPC_OSQP_Ackermann.py',
            output = 'screen'),
        Node(
            package = 'mpc_ackermann',
            executable = 'path_drawer.py',
            output = 'screen'),
        Node(
            package = 'mpc_ackermann',
            executable = 'tf_broadcaster',
            output = 'screen',
            arguments=['r1'])  # argv[1] en tu nodo
    ])