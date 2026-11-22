import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mini_sentry')
    config_file = os.path.join(pkg_share, 'config', 'sentry.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    simulate_damage = LaunchConfiguration('simulate_damage')
    match_duration = LaunchConfiguration('match_duration')
    defense_time = LaunchConfiguration('defense_time')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='回放 bag 时设成 true'),
        DeclareLaunchArgument('simulate_damage', default_value='true',
                              description='裁判仿真是否模拟掉血'),
        DeclareLaunchArgument('match_duration', default_value='420.0',
                              description='一局时长（秒），演示 60 秒规则时设 80 左右'),
        DeclareLaunchArgument('defense_time', default_value='60.0',
                              description='最后多少秒回防'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否顺带打开 RViz2'),

        Node(
            package='mini_sentry',
            executable='chassis_sim',
            name='chassis_sim',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='mini_sentry',
            executable='referee_sim',
            name='referee_sim',
            output='screen',
            parameters=[config_file,
                        {'use_sim_time': use_sim_time,
                         'simulate_damage': simulate_damage,
                         'match_duration': match_duration}],
        ),
        Node(
            package='mini_sentry',
            executable='goto_server',
            name='goto_server',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='mini_sentry',
            executable='sentry_brain',
            name='sentry_brain',
            output='screen',
            parameters=[config_file,
                        {'use_sim_time': use_sim_time,
                         'defense_time': defense_time}],
        ),

        # 静态 TF：雷达装在底盘前上方
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='livox_static_tf',
            arguments=['--x', '0.12', '--y', '0.0', '--z', '0.18',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
        ),

        Node(
            package='mini_sentry',
            executable='tf_demo',
            name='tf_demo',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # 只在 rviz:=true 时启动
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'sentry.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])
