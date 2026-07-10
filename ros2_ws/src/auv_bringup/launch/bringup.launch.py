"""AUV yazilim yigininin tamami.

    ros2 launch auv_bringup bringup.launch.py
    ros2 launch auv_bringup bringup.launch.py with_camera:=false   # tezgah testi

Proje kok dizini varsayilani ~/auv (Jetson'a rsync hedefi); farkliysa:
    ros2 launch auv_bringup bringup.launch.py project_root:=/opt/auv
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('auv_bringup')
    params = os.path.join(pkg_share, 'config', 'auv_params.yaml')

    project_root = LaunchConfiguration('project_root')
    with_camera = LaunchConfiguration('with_camera')

    def npath(sub):
        # project_root altindaki config yollari
        from launch.substitutions import PathJoinSubstitution
        return PathJoinSubstitution([project_root, sub])

    nodes = [
        Node(package='auv_mav_bridge', executable='bridge_node',
             name='mav_bridge', parameters=[params], output='screen'),
        Node(package='auv_vertical_state', executable='vertical_state_node',
             name='vertical_state', parameters=[params], output='screen'),
        Node(package='auv_lane_follow', executable='lane_follow_node',
             name='lane_follow', parameters=[params], output='screen'),
        Node(package='auv_dead_reckoning', executable='dr_node',
             name='dead_reckoning',
             parameters=[params,
                         {'speed_table_csv': npath('config/speed_table.csv')}],
             output='screen'),
        Node(package='auv_mission', executable='mission_node',
             name='mission',
             parameters=[params,
                         {'missions_dir': npath('config/missions'),
                          'speed_table_csv': npath('config/speed_table.csv')}],
             output='screen'),
        Node(package='auv_video', executable='video_stream_node',
             name='video_stream', parameters=[params], output='screen',
             condition=IfCondition(with_camera)),
    ]

    # realsense2_camera kuruluysa D435 surucusu
    realsense = None
    try:
        rs_share = get_package_share_directory('realsense2_camera')
        realsense = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rs_share, 'launch', 'rs_launch.py')),
            launch_arguments={
                'rgb_camera.color_profile': '640x480x30',
                'enable_depth': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'camera_namespace': '/',
                'camera_name': 'camera',
            }.items(),
            condition=IfCondition(with_camera),
        )
    except Exception:
        pass  # kamera paketi yoksa (tezgah/CI) sessiz gec

    args = [
        DeclareLaunchArgument('project_root',
                              default_value=os.path.expanduser('~/auv'),
                              description='Depo kok dizini (config/ barindirir)'),
        DeclareLaunchArgument('with_camera', default_value='true'),
    ]
    ld = LaunchDescription(args + nodes)
    if realsense is not None:
        ld.add_action(realsense)
    return ld
