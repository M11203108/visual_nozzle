# ~/newjasmine_ws/src/<你想放的套件>/launch/all_in_one_launch.py
# -------------------------------------------------------------
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description() -> LaunchDescription:
    # ========== 取得兩個套件的 share 目錄 ==========
    thermal_share = get_package_share_directory('thermal_camera')
    fire_share    = get_package_share_directory('rs485_fire')

    # ========== 1) thermal_camera/fire_all_launch.py ==========
    fire_all = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(thermal_share, 'launch', 'fire_all_launch.py')
        ),
        launch_arguments={}
    )

    final_spray_with_delay = TimerAction(
        period=10.0,          # 10 秒後執行
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(fire_share, 'launch', 'final_spray_launch.py')
                ),
                launch_arguments={}
            )
        ]
    )

    # ========== 回傳 LaunchDescription ==========
    return LaunchDescription([
        fire_all,
        final_spray_with_delay
    ])
