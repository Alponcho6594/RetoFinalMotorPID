import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(

        get_package_share_directory('signal_sent'),
        'config',
        'params.yaml'
    )

    generation_node = Node(
        package = 'signal_sent',
        executable = 'setpoint',
        output = 'screen',
        parameters = [config]
    )

    return LaunchDescription([generation_node])