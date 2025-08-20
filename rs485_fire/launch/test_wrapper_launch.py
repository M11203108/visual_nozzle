from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    fire_share = get_package_share_directory('rs485_fire')

    # 先定義包含的 all_in_one launch
    all_in_one_launch = os.path.join(fire_share, 'launch', 'new_all_launch.py')
    all_in_one_desc = IncludeLaunchDescription(PythonLaunchDescriptionSource(all_in_one_launch))

    # 改為監聽 all_in_one 中某個具體節點（例如 shutdown_trigger_node）
    shutdown_trigger_node_name = 'shutdown_trigger_node'  # 🔁 確保此名稱一致

    # 建立 handler，當 shutdown_trigger_node 結束後才印訊息
    handler = RegisterEventHandler(
        OnProcessExit(
            target_action=None,  # 因為 target_action 不支援 IncludeLaunchDescription
            on_exit=[LogInfo(msg="✅ all_in_one_launch.py 已關閉，但上層仍在！")]
        )
    )

    return LaunchDescription([
        LogInfo(msg="🚀 啟動上層 test_wrapper_launch"),
        all_in_one_desc,
        handler
    ])
