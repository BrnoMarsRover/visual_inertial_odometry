# =============================================================
# Project:   visual_inertial_odometry
# File:      launch/sqrtVINS.launch.py
# Author:    Martin Kriz
# -------------------------------------------------------------
# Description: Launch file for sqrtVINS with RealSense D435 + Pixhawk IMU
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
    image_topic  = LaunchConfiguration('image_topic').perform(context)
    imu_topic    = LaunchConfiguration('imu_topic').perform(context)

    indexed_robot_name = f'{robot_name}_{robot_number}' if robot_number else robot_name

    config_path = os.path.join(
        get_package_share_directory('visual_inertial_odometry'),
        'config',
        'f450_sqrtVINS',
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
            {'use_stereo': False},
            {'max_cameras': 1},
            {'save_total_state': LaunchConfiguration('save_total_state')},
            {'config_path': config_path},
        ],
        remappings=[
            ('/sqrtvins/image_raw', f'/{indexed_robot_name}{image_topic}'),
            ('/sqrtvins/imu',       f'/{indexed_robot_name}{imu_topic}'),
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
            name='image_topic',
            default_value='/sensors/realsense_d435/realsense_camera_node/infra1/image_rect_raw',
            description='Camera image topic relative to robot prefix. (sensor_msgs/Image)',
        ),
        DeclareLaunchArgument(
            name='imu_topic',
            default_value='/aircraft/imu',
            description='IMU topic relative to robot prefix. (sensor_msgs/Imu)',
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
        OpaqueFunction(function=launch_setup),
    ])
