# =============================================================
# Project:   visual_inertial_odometry
# File:      launch/isaac_ros_visual_slam.launch.py
# Author:    Martin Kriz
# Created:   2025-10-26
# -------------------------------------------------------------
# Description: Launch file for Isaac ROS Visual SLAM node
# Notes: Launch file is based on Milos Cihlar example package
# =============================================================

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
from typing import Optional, List


def launch_setup(context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:

    vio_dir_config = os.path.join(get_package_share_directory('visual_inertial_odometry'), 'config')

    # Create the launch configuration variables
    robot_name = LaunchConfiguration('robot_name')
    robot_number = LaunchConfiguration('robot_number')

    config = LaunchConfiguration('config')

    image0_topic = LaunchConfiguration('image0_topic')
    camera0_topic = LaunchConfiguration('camera0_topic')
    image1_topic = LaunchConfiguration('image1_topic')
    camera1_topic = LaunchConfiguration('camera1_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    imu_fusion = LaunchConfiguration('imu_fusion')
    slam_localization = LaunchConfiguration('slam_localization')

    indexed_robot_name = [robot_name.perform(context), '_', robot_number.perform(context)] if robot_number.perform(context) else [robot_name.perform(context)]
    indexed_robot_name = ''.join(indexed_robot_name)

    camera_optical_frames = [
        f'{indexed_robot_name}_realsense_d435_infra1_optical_frame',
        f'{indexed_robot_name}_realsense_d435_infra2_optical_frame',
    ]

    config_substitutions = {
            'enable_imu_fusion': imu_fusion.perform(context),
            'enable_slam_localization': slam_localization.perform(context),
            'base_frame': f'{indexed_robot_name}_base_link',
            'imu_frame':  f'{indexed_robot_name}_base_link',
    }

    config_dir = os.path.join(vio_dir_config, robot_name.perform(context))
    os.chdir(config_dir)

    configured_config_file = RewrittenYaml(
        source_file = config,
        root_key = indexed_robot_name + '/vio_isaac',
        param_rewrites = config_substitutions,
        convert_types = True)

    visual_slam_node = ComposableNode(
        package = 'isaac_ros_visual_slam',
        name = 'visual_slam_node',
        namespace = indexed_robot_name + '/vio_isaac',
        plugin = 'nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters = [configured_config_file, {'camera_optical_frames': camera_optical_frames}],
        remappings = [
            ('visual_slam/image_0',
            f'/{indexed_robot_name}{image0_topic.perform(context)}'),
            
            ('visual_slam/camera_info_0',
            f'/{indexed_robot_name}{camera0_topic.perform(context)}'),
            
            ('visual_slam/image_1',
            f'/{indexed_robot_name}{image1_topic.perform(context)}'),
            
            ('visual_slam/camera_info_1',
            f'/{indexed_robot_name}{camera1_topic.perform(context)}'),

            ('visual_slam/imu', 
            f'/{indexed_robot_name}{imu_topic.perform(context)}'),
        ],
    )

    visual_slam_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='visual_slam_launch_container',
        namespace=indexed_robot_name + '/vio_isaac',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )
    return [LaunchDescription([visual_slam_container])]


def generate_launch_description():
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            name = 'robot_name',
            description = 'Robot name (ex: "robot").'
        ),
        DeclareLaunchArgument(
            name = 'robot_number',
            default_value = '',
            description = 'Robot number, which will be appended to the robot name if defined ("<name>_<number>", '
                        'default is empty).'
        ),
        DeclareLaunchArgument(
            name = 'config',
            description = 'Algorithm configuration.'
        ),
        DeclareLaunchArgument(
            name = 'image0_topic',
            default_value = '/sensors/realsense_d435/realsense_camera_node/infra1/image_rect_raw',
            description = 'Input topic from camera 0. (sensor_msgs/Image)'
        ),
        DeclareLaunchArgument(
            name = 'camera0_topic',
            default_value = '/sensors/realsense_d435/realsense_camera_node/infra1/camera_info',
            description = 'Input topic from camera 0.(sensor_msgs/CameraInfo)'
        ),
        DeclareLaunchArgument(
            name = 'image1_topic',
            default_value = '/sensors/realsense_d435/realsense_camera_node/infra2/image_rect_raw',
            description = 'Input topic from camera 1. (sensor_msgs/Image)'
        ),
        DeclareLaunchArgument(
            name = 'camera1_topic',
            default_value = '/sensors/realsense_d435/realsense_camera_node/infra2/camera_info',
            description = 'Input topic from camera 1. (sensor_msgs/CameraInfo)'
        ),
        DeclareLaunchArgument(
            name = 'imu_topic',
            default_value = '/aircraft/imu',
            description = 'Input topic from IMU. (sensor_msgs/Imu)'
        ),
        DeclareLaunchArgument(
            name = 'imu_fusion',
            default_value = 'True',
            description = 'Enable IMU fusion. (False = disbled)'
        ),
        DeclareLaunchArgument(
            name = 'slam_localization',
            default_value = 'True',
            description = 'Activate SLAM localization. (False = disabled)'
        ),
        # Perform the launch setup
        OpaqueFunction(function=launch_setup)
    ])
