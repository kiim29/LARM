import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='visualization',
            arguments=['-d', [os.path.join('/home/bot/ros2_ws/LARM-kim-nathan/grp_data/for_rvizz/config_rviz2_for_chall2.rviz')]]
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            prefix='gnome-terminal -x',
            remappings=[
                ('/cmd_vel', '/multi/cmd_teleop')
            ]    
        )
        # Node(
        #     package='nav2_bringup',
        #     executable='navigation_launch',
        #     name='navigation_launch'
        # )
    ])