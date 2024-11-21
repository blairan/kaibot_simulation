from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_path
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = 'xiaoKaiRobot'
    rviz_name = 'lds_robot.rviz'
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # urdf
    path2usdf = get_package_share_path('kaibot_description') / 'urdf' / 'xiao_kai_robot.urdf'
    
    #xacro
    # path2usdf = get_package_share_path('xiaoKaiRobot') / 'urdf' / 'xacro'/ 'xiao_kai_robot.urdf.xacro'
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # 解柝xacro為urdf的參數
         parameters=[{
        'robot_description': ParameterValue(
            Command(['xacro ', str(path2usdf)]), value_type=str) 
        }])
        
    robot_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', os.path.join(pkg_share, "rviz/lds_robot.rviz")],
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
       robot_state_publisher,
       robot_joint_state_publisher,
       rviz2
    ])