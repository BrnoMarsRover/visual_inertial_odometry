# =============================================================
# Project:   visual_inertial_odometry
# File:      launch/sqrtVINS.launch.py
# Author:    Martin Kriz
# -------------------------------------------------------------
# Description: Launch file for sqrtVINS (mono and stereo)
# Topics (camera + IMU) are read directly from kalibr yaml files in config_dir.
# =============================================================

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
from typing import Optional, List


def launch_setup(context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:

    robot_name   = LaunchConfiguration('robot_name').perform(context)
    robot_number = LaunchConfiguration('robot_number').perform(context)
    config_dir   = LaunchConfiguration('config_dir').perform(context)

    indexed_robot_name = f'{robot_name}_{robot_number}' if robot_number else robot_name

    config_path = os.path.join(
        get_package_share_directory('visual_inertial_odometry'),
        'config',
        config_dir,
        'estimator_config.yaml',
    )

    if not os.path.isfile(config_path):
        return [LogInfo(msg=f'ERROR: config not found: {config_path}')]

    node = Node(
        package='ov_srvins',
        executable='run_subscribe_msckf',
        namespace=indexed_robot_name + '/ov_srvins',
        output='screen',
        parameters=[
            {'verbosity': LaunchConfiguration('verbosity')},
            {'save_total_state': LaunchConfiguration('save_total_state')},
            {'config_path': config_path},
        ],
        remappings=[
            ('/f450_1/oak/imu/data',        LaunchConfiguration('topic_imu')),
            ('/f450_1/oak/left/image_raw',   LaunchConfiguration('topic_camera0')),
            ('/f450_1/oak/right/image_raw',  LaunchConfiguration('topic_camera1')),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz_enable')),
        arguments=[
            '-d' + os.path.join(
                get_package_share_directory('ov_srvins'), 'launch', 'display_ros2.rviz'
            ),
            '--ros-args', '--log-level', 'warn',
        ],
    )

    return [node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_name',
            description='Robot name (ex: "f450").',
        ),
        DeclareLaunchArgument(
            name='robot_number',
            default_value='',
            description='Robot number appended to robot name if set ("f450_1"), default is empty.',
        ),
        DeclareLaunchArgument(
            name='config_dir',
            description='Config directory name under share/visual_inertial_odometry/config/ (ex: "f450_sqrtVINS_stereo").',
        ),
        DeclareLaunchArgument(
            name='rviz_enable',
            default_value='false',
            description='Enable rviz visualization.',
        ),
        DeclareLaunchArgument(
            name='verbosity',
            default_value='INFO',
            description='ALL, DEBUG, INFO, WARNING, ERROR, SILENT',
        ),
        DeclareLaunchArgument(
            name='save_total_state',
            default_value='false',
            description='Record total state with calibration and features to txt file.',
        ),
        DeclareLaunchArgument(
            name='topic_imu',
            default_value='/f450_1/sensors/oak_d_pro_w/imu/data',
            description='IMU topic to subscribe to.',
        ),
        DeclareLaunchArgument(
            name='topic_camera0',
            default_value='/f450_1/sensors/oak_d_pro_w/left/image_raw',
            description='Left camera image topic.',
        ),
        DeclareLaunchArgument(
            name='topic_camera1',
            default_value='/f450_1/sensors/oak_d_pro_w/right/image_raw',
            description='Right camera image topic.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
