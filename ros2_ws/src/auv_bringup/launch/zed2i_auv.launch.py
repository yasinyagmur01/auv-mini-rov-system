# AUV - ZED 2i baslatma dosyasi
#
# Resmi zed_camera.launch.py'yi AUV'a ozel parametre dosyasiyla sarar ve
# 3D harita depolama dugumunu (map_saver) baslatir.
#
# Yol sahipligi: harita dizini SADECE 'maps_dir' argumaniyla belirlenir.
# area_file_path, zed_camera.launch.py'nin en yuksek oncelikli
# 'param_overrides' mekanizmasiyla maps_dir'den turetilir (YAML'a yazilmaz).
#
# Kullanim:
#   ros2 launch auv_bringup zed2i_auv.launch.py                     # Docker: /data/maps
#   ros2 launch auv_bringup zed2i_auv.launch.py maps_dir:=/home/qayra/auv/maps
#   ros2 launch auv_bringup zed2i_auv.launch.py enable_gnss:=true   # F9P entegre olunca

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('auv_bringup')
    zed_wrapper_share = get_package_share_directory('zed_wrapper')

    default_override = os.path.join(bringup_share, 'config', 'zed2i_auv.yaml')

    override_path_arg = DeclareLaunchArgument(
        'override_path',
        default_value=default_override,
        description='ZED node parametrelerini ezen YAML dosyasi',
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='zed',
        description="Topic ad alani: /<camera_name>/zed_node/...",
    )

    maps_dir_arg = DeclareLaunchArgument(
        'maps_dir',
        default_value='/data/maps',
        description='Harita ciktilari: .area (lokalizasyon) + .ply (3D harita)',
    )

    enable_gnss_arg = DeclareLaunchArgument(
        'enable_gnss',
        default_value='false',
        description='GNSS fuzyonu (global localization); F9P GPS entegre olunca true',
    )

    # SDK 5.4.0'da deneysel olarak dogrulanmis takas (2026-07-05):
    #   GEN_1 -> spatial mapping CALISIR, area memory URETMEZ ('FILE EMPTY')
    #   GEN_3 -> area memory CALISIR (relocalization), spatial mapping BOS doner
    # Haritalama gorevlerinde GEN_1, onceden haritalanmis ortamda kendini
    # konumlama gorevlerinde tracking_mode:=GEN_3 kullanin.
    tracking_mode_arg = DeclareLaunchArgument(
        'tracking_mode',
        default_value='GEN_1',
        description="Pozisyon takibi modu: 'GEN_1' (3D haritalama) | 'GEN_3' (area relocalization)",
    )

    # En yuksek oncelikli param_overrides (zed_camera.launch.py 'highest priority').
    # KRITIK: area dosyasi YALNIZ GEN_3'te yuklenir/kaydedilir. GEN_1'de area
    # dosyasi yuklemek relocalization aramasini (SEARCHING) sonsuza dek surdurur
    # ve spatial mapping HIC nokta entegre etmez (tani 2026-07-05:
    # pose/status.spatial_memory_status=SEARCHING iken fused_cloud hep 0).
    # GEN_1'de area_memory TAMAMEN kapali: USB kopmasi/tracking kaybi sonrasi
    # RAM ici area memory de SEARCHING kilidi kuruyor (2026-07-05 14:27 vakasi:
    # USB disconnect -> spatial_memory_status=SEARCHING -> harita 198k'da dondu).
    # GEN_1 relocalization zaten calismadigindan kayip yalniz kilitlenmedir.
    param_overrides = PythonExpression([
        "'pos_tracking.pos_tracking_mode:=", LaunchConfiguration('tracking_mode'),
        "' + (';pos_tracking.area_file_path:=",
        LaunchConfiguration('maps_dir'),
        "/auv_area.area;pos_tracking.save_area_memory_on_closing:=true'",
        " if '", LaunchConfiguration('tracking_mode'), "' == 'GEN_3'",
        " else ';pos_tracking.save_area_memory_on_closing:=false"
        ";pos_tracking.area_memory:=false')",
    ])

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_share, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': LaunchConfiguration('camera_name'),
            'ros_params_override_path': LaunchConfiguration('override_path'),
            'enable_gnss': LaunchConfiguration('enable_gnss'),
            'param_overrides': param_overrides,
        }.items(),
    )

    map_saver_node = Node(
        package='auv_bringup',
        executable='map_saver.py',
        name='map_saver',
        output='screen',
        parameters=[{
            'cloud_topic': PythonExpression(
                ["'/", LaunchConfiguration('camera_name'),
                 "/zed_node/mapping/fused_cloud'"]),
            'output_dir': LaunchConfiguration('maps_dir'),
            'file_prefix': 'fused_map',
            'autosave_period': 60.0,
        }],
    )

    return LaunchDescription([
        override_path_arg,
        camera_name_arg,
        maps_dir_arg,
        enable_gnss_arg,
        tracking_mode_arg,
        zed_launch,
        map_saver_node,
    ])
