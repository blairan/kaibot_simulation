import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_name_in_model = 'xiao_kai_robot'
    package_name = 'xiaoKaiRobot'
    urdf_name = "xiao_kai_robot.urdf"
    gazebo_world = os.path.join(get_package_share_directory('kaibot_description'), 'world', 'chinic_multi2.world')

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    # Start Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world],
        output='screen'
    )

    # Start the robot state publisher with use_sim_time parameter
    start_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'use_sim_time': True}],  # 將 use_sim_time 作為參數傳遞
        arguments=[urdf_model_path]
    )

    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-file', urdf_model_path],
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, "rviz/xiao_kai_robot_base_conf.rviz")],
        name='rviz2',
        output='screen')
    
    move_display_cmd = Node(
        package="xiaoKaiRobot",
        executable="move_speed_node",
        name="move_speed_node",
        output='screen',
    )

    # Add actions to the launch description
    ld.add_action(move_display_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher)
    ld.add_action(rviz2)
    
    return ld
