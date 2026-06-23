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
import yaml
from typing import Optional, List


def _load_opencv_yaml(path: str) -> dict:
    """Load OpenCV-style YAML (strips %YAML:x.x directive that PyYAML rejects)."""
    with open(path) as f:
        content = f.read()
    lines = [l for l in content.splitlines() if not l.startswith('%YAML')]
    return yaml.safe_load('\n'.join(lines))


def launch_setup(context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name   = LaunchConfiguration('robot_name').perform(context)
    robot_number = LaunchConfiguration('robot_number').perform(context)
    config_dir   = LaunchConfiguration('config_dir').perform(context)

    indexed_robot_name = [robot_name, '_', robot_number] if robot_number else [robot_name]
    indexed_robot_name = ''.join(indexed_robot_name)

    pkg_share = get_package_share_directory('visual_inertial_odometry')
    config_base = os.path.join(pkg_share, 'config', config_dir)

    config_path = os.path.join(config_base, 'estimator_config.yaml')
    if not os.path.isfile(config_path):
        return [LogInfo(msg=f'ERROR: config not found: {config_path}')]

    estimator_cfg = _load_opencv_yaml(config_path)
    imu_yaml_path = os.path.join(config_base, estimator_cfg['relative_config_imu'])
    cam_yaml_path = os.path.join(config_base, estimator_cfg['relative_config_imucam'])

    imu_topic_yaml  = _load_opencv_yaml(imu_yaml_path)['imu0']['rostopic']
    cam_yaml        = _load_opencv_yaml(cam_yaml_path)
    cam0_topic_yaml = cam_yaml['cam0']['rostopic']
    cam1_topic_yaml = cam_yaml.get('cam1', {}).get('rostopic')

    topic_imu     = LaunchConfiguration('topic_imu').perform(context)
    topic_camera0 = LaunchConfiguration('topic_camera0').perform(context)
    topic_camera1 = LaunchConfiguration('topic_camera1').perform(context)

    remappings = [
        (imu_topic_yaml,  topic_imu  or imu_topic_yaml),
        (cam0_topic_yaml, topic_camera0 or cam0_topic_yaml),
    ]
    if cam1_topic_yaml:
        remappings.append((cam1_topic_yaml, topic_camera1 or cam1_topic_yaml))

    node = Node(
        package='ov_srvins',
        executable='run_subscribe_msckf',
        namespace=indexed_robot_name + '/ov_srvins',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'verbosity': LaunchConfiguration('verbosity')},
            {'save_total_state': LaunchConfiguration('save_total_state')},
            {'config_path': config_path},
        ],
        remappings=remappings,
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
            name='use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true.'
        ),
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
            default_value='',
            description='Override IMU topic. Empty = use rostopic from kalibr_imu_chain.yaml.',
        ),
        DeclareLaunchArgument(
            name='topic_camera0',
            default_value='',
            description='Override left camera topic. Empty = use rostopic from kalibr_imucam_chain.yaml.',
        ),
        DeclareLaunchArgument(
            name='topic_camera1',
            default_value='',
            description='Override right camera topic. Empty = use rostopic from kalibr_imucam_chain.yaml.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
