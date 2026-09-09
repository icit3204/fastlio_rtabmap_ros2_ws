import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    startup_guard_enabled = LaunchConfiguration('startup_freshness_guard_enabled')
    startup_max_lidar_age = LaunchConfiguration('startup_max_lidar_age_sec')
    startup_fresh_groups = LaunchConfiguration('startup_fresh_consecutive_groups')
    lidar_topic = LaunchConfiguration('lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_startup_guard_enabled_cmd = DeclareLaunchArgument(
        'startup_freshness_guard_enabled', default_value='false',
        description='Contain stale synchronized startup groups before FAST-LIO admission'
    )
    declare_startup_max_lidar_age_cmd = DeclareLaunchArgument(
        'startup_max_lidar_age_sec', default_value='0.5',
        description='Maximum synchronized lidar_end_time age for startup admission'
    )
    declare_startup_fresh_groups_cmd = DeclareLaunchArgument(
        'startup_fresh_consecutive_groups', default_value='10',
        description='Consecutive fresh synchronized groups required for startup admission'
    )
    declare_lidar_topic_cmd = DeclareLaunchArgument(
        'lidar_topic', default_value='/livox/lidar',
        description='FAST-LIO LiDAR input topic; generic default remains /livox/lidar'
    )
    declare_imu_topic_cmd = DeclareLaunchArgument(
        'imu_topic', default_value='/livox/imu',
        description='FAST-LIO IMU input topic; generic default remains /livox/imu'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time,
                     'common.lid_topic': lidar_topic,
                     'common.imu_topic': imu_topic,
                     'startup_freshness_guard_enabled': startup_guard_enabled,
                     'startup_max_lidar_age_sec': startup_max_lidar_age,
                     'startup_fresh_consecutive_groups': startup_fresh_groups}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_startup_guard_enabled_cmd)
    ld.add_action(declare_startup_max_lidar_age_cmd)
    ld.add_action(declare_startup_fresh_groups_cmd)
    ld.add_action(declare_lidar_topic_cmd)
    ld.add_action(declare_imu_topic_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
