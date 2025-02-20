from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Launch RViz'
    )

    config = os.path.join(
        get_package_share_directory('fastlivo2'),
        'config',
        'avia.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory('fastlivo2'),
        'rviz',
        'fast_livo2.rviz'
    )

    # Load parameters
    slam_node = Node(
        package='fastlivo2',
        executable='fastlivo_mapping',
        output='screen',
        parameters=[config]
    )

    # RViz node
    rviz_node = GroupAction([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])

    # Image transport republish node
    image_republish = Node(
        package='image_transport',
        executable='republish',
        name='republish',
        arguments=['compressed', 'in:=/left_camera/image', 'raw', 'out:=/left_camera/image'],
        output='screen',
        respawn=True
    )

    return LaunchDescription([
        rviz_arg,
        slam_node,
        rviz_node,
        image_republish
    ])
