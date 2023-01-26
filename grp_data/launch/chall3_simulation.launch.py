import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    tbot_sim_path = get_package_share_directory('tbot_sim')
    launch_file_dir = os.path.join(tbot_sim_path, 'launch')
    slam_toolbox_path = get_package_share_directory('slam_toolbox')
    launch2_file_dir = os.path.join(slam_toolbox_path, 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/challenge-1.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch2_file_dir, '/online_sync_launch.py']),
            launch_arguments={
                'use_sim_time': 'False'
            }.items()
        ),
        Node(
            package='grp_data',
            executable='scan_echo',
            name='scan'
        ),
        Node(
            package='grp_data',
            executable='reactive_move',
            name='reactive_move',
            remappings=[
                ('/commands/velocity', '/multi/cmd_nav')
            ]    
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='visualization',
            arguments=['-d', [os.path.join('/home/bot/ros2_ws/LARM-kim-nathan/grp_data/for_rvizz/config_rviz2_for_chall2_sim.rviz')]]
        ),
        Node(
            package='tbot_pytools',
            executable='multiplexer',
            name='multiplexer'
        )
    ])