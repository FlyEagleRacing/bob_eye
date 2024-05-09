from launch import LaunchDescription, launch_description_sources
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtk_observer',
            executable='rtk_observer_exe',
            output='screen',
        ),
        Node(
            package='bob_eye_filter',
            executable='bob_eye_filter_exe',
            output='screen',
        ),
    ])