import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = get_package_share_directory('xiaokaibot_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    map_path = LaunchConfiguration('map', default=os.path.join('/home/ctk/kaibot_ws/src/xiaokaibot_nav2', 'maps', 'my_slam_box3.yaml'))
    param_file_name = 'nav2_params.yaml'
    param_file_path = LaunchConfiguration('param_file', default=os.path.join(
        get_package_share_directory('xiaokaibot_nav2'), 'params', param_file_name))
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_path,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_file_path,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_path,
                'use_sim_time': use_sim_time,
                'params_file': param_file_path}.items()),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])