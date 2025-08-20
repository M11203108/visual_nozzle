from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, RegisterEventHandler, Shutdown, ExecuteProcess
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 🟢 立即啟動的三個感測節點
    ir_camera_node = Node(
        package='thermal_camera',
        executable='thermal_camera_node',
        name='thermal_camera',
        output='screen'
    )

    thermal_to_rgb = Node(
        package='thermal_camera',
        executable='thermal_to_rgb_pixel_node.py',
        name='thermal_to_rgb_pixel_node',
        output='screen'
    )

    fire_detection = Node(
        package='thermal_camera',
        executable='fire_detection.py',
        name='fire_detection',
        output='screen'
    )

    # ⏱️ 延遲啟動：5 秒後啟動軌跡規劃節點
    spray_planner_delay = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rs485_fire',
                executable='spray_trajectories_final_node.py',
                name='spray_planner',
                output='screen'
            )
        ]
    )

    # ⏱️ 延遲啟動：10 秒後啟動噴灑執行節點
    spray_executor_delay = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rs485_fire',
                executable='spray_executor_node.py',
                name='spray_executor',
                output='screen'
            )
        ]
    )

    # ✅ 改為 ExecuteProcess 啟動 shutdown_trigger_node（避免其成為 required）
    shutdown_trigger = ExecuteProcess(
        cmd=['ros2', 'run', 'rs485_fire', 'shutdown_trigger_node.py'],
        name='shutdown_trigger_node',
        output='screen',
        shell=False
    )

    # 🔚 shutdown_trigger_node 結束時，自動觸發關閉本 launch（不影響上層）
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=shutdown_trigger,
            on_exit=[Shutdown(reason="✅ all_in_one_launch 自動結束 (上層不受影響)")]
        )
    )

    return LaunchDescription([
        ir_camera_node,
        thermal_to_rgb,
        fire_detection,
        spray_planner_delay,
        spray_executor_delay,
        shutdown_trigger,
        shutdown_handler
    ])
