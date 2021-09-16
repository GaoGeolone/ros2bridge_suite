from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import os
import launch_ros.actions
import pathlib

def generate_launch_description():
    # parameters_file_path = str(pathlib.Path(__file__).parents[1]) # get current path and go one level up
    # parameters_file_path += '/config/' + parameters_file_name
    # parameters_file_path += './' + 'default.yaml' 
    # print(parameters_file_path)
    return LaunchDescription([
         launch_ros.actions.Node(
            name='rosbridge_tcp',
            package='rosbridge_server',
            node_executable='rosbridge_tcp',
            output='screen',
            # parameters=[
            #     parameters_file_path
            # ],
         ),
         launch_ros.actions.Node(
            name='rosapi',
            package='rosapi',
            node_executable='rosapi_node',
            output='screen',
            # parameters=[
            #     parameters_file_path
            # ],
         ),
    ]
    )