import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    package_dir = get_package_share_directory('task1_simulation')

    world_file = os.path.join(
        package_dir,
        'worlds',
        'corridor.world'
    )

    turtlebot_model = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )

    # Start Gazebo with ROS plugins
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # Spawn TurtleBot Burger
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'burger',
            '-file', turtlebot_model,
            '-x', '-8.0',
            '-y', '0.0',
            '-z', '0.05'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot
    ])
