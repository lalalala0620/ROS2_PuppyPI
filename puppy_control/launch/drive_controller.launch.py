import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # drive_param_dir = LaunchConfiguration(
    #     'drive_param_dir',
    #     default=os.path.join(
    #         get_package_share_directory('evpi_drive'),
    #         'config','evpi' + '.yaml'))

    return LaunchDescription([

        # DeclareLaunchArgument(
        #     'drive_param_dir',
        #     default_value=drive_param_dir,
        #     description='Full path to evpi parameter file to load'),

        Node(
            package='puppy_control',
            executable='puppy_control',
            name='puppy_control',
            # parameters=[drive_param_dir],
            # remappings=[
            # 	('auto_brake_servo', '/auto_brake_servo'),
            # 	('auto_acker_cmd', '/auto_acker_cmd')
            # ],
            output='screen'),

    ])