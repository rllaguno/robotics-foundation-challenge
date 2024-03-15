import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('reto'),
        'config',
        'params.yaml'
    )

    setpoint_node = Node(
        package = 'reto',
        executable = 'Setpoint',
        output = 'screen', 
        parameters = [config]
    )

    rqt_graph_node = Node(
        package = 'rqt_graph',
        executable = 'rqt_graph',
        output = 'screen',
    )

    l_d = LaunchDescription([setpoint_node, rqt_graph_node])
    return l_d