from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    gazebo_params_file = os.path.join(pkg_share, 'config', 'gazebo_params.yaml')
    worlds_file = os.path.join(pkg_share, 'worlds', 'obstacles.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]), 
        launch_arguments={
            'world': worlds_file,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_robot'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    map_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_file': '/home/thang/ros2_workspace/src/robot_description/maps/map.yaml'}]
    )

    # nav2_node = Node(
    #     package="nav2_bringup",
    #     executable="bringup.launch.py",
    #     parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')]
    # )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_node,
        rviz_node,
        # nav2_node,
    ])
