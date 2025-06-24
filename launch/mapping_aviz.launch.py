#!/usr/bin/python3
# -- coding: utf-8 --**

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    # Find path
    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")

    #Load parameters
    avia_config_cmd = os.path.join(config_file_dir, "avia.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_pinhole.yaml")

    # Param use_rviz
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Whether to launch Rviz2",
    )

    avia_config_arg = DeclareLaunchArgument(
        'avia_params_file',
        default_value=avia_config_cmd,
        description='Full path to the ROS2 parameters file to use for fast_livo2 nodes',
    )

    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_config_cmd,
        description='Full path to the ROS2 parameters file to use for vikit_ros nodes',
    )

    # https://github.com/ros-navigation/navigation2/blob/1c68c212db01f9f75fcb8263a0fbb5dfa711bdea/nav2_bringup/launch/navigation_launch.py#L40
    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    avia_params_file = LaunchConfiguration('avia_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    use_respawn = LaunchConfiguration('use_respawn')

    return LaunchDescription([
        use_rviz_arg,
        avia_config_arg,
        camera_config_arg,
        use_respawn_arg,

        # play ros2 bag
        # ExecuteProcess(
        #     cmd=[['ros2 bag play ', '~/datasets/Retail_Street ', '--clock ', "-l"]], 
        #     shell=True
        # ),

        # republish compressed image to raw image
        # https://robotics.stackexchange.com/questions/110939/how-do-i-remap-compressed-video-to-raw-video-in-ros2
        # ros2 run image_transport republish compressed raw --ros-args --remap in:=/left_camera/image --remap out:=/left_camera/image
        Node(
            package="image_transport",
            executable="republish",
            name="republish",
            arguments=[ # Array of strings/parametric arguments that will end up in process's argv
                'compressed', 
                'raw',
            ],
            remappings=[
                ("in",  "/left_camera/image"), 
                ("out", "/left_camera/image")
            ],
            output="screen",
            respawn=use_respawn,
        ),
        
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                avia_params_file,
                camera_params_file,
            ],
            # https://docs.ros.org/en/humble/How-To-Guides/Getting-Backtraces-in-ROS-2.html
            prefix=[
                # ("gdb -ex run --args"),
                # ("valgrind --log-file=./valgrind_report.log --tool=memcheck --leak-check=full --show-leak-kinds=all -s --track-origins=yes --show-reachable=yes --undef-value-errors=yes --track-fds=yes")
            ],
            output="screen"
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),
    ])
