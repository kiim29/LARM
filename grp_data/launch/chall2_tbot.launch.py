import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    tbot_start_path = get_package_share_directory('tbot_start')
    launch_file_dir = os.path.join(tbot_start_path, 'launch')
    slam_toolbox_path = get_package_share_directory('slam_toolbox')
    launch2_file_dir = os.path.join(slam_toolbox_path, 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/minimal.launch.py'])
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
            package='grp_data',
            executable='realsense',
            name='realsense'
        ),
        # Node(
        #     package='grp_data',
        #     executable='cola_detect',
        #     name='cola_detect'
        # ),
        # Node(
        #     package='grp_data',
        #     executable='cherry_detect',
        #     name='cherry_detect'
        # ),
        Node(
            package='grp_data',
            executable='bottles_detect',
            name='bottles_detect'
        )
    ])