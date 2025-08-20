from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, RegisterEventHandler, Shutdown, ExecuteProcess, LogInfo
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 🟢 立即啟動：三個感測節點
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

    # 🧭 對正節點（完成後才開後續）
    alignment = Node(
        package='rs485_fire',
        executable='alignment_controller_node.py',
        name='alignment_stepper',
        output='screen',
        parameters=[{
            # 需要可在這裡微調
            'deadband_deg': 1.0,
            'required_zero_cycles': 3,
            'max_step_deg': 20.0,
            'post_move_min_sec': 0.5,
            'stationary_hold_sec': 0.4,
            'angvel_thresh': 0.02,
            'stability_deg': 1.0,
            'stability_window': 10,
            # 'input_bias_deg': 6.0,  # 若有固定偏差可開啟
        }]
    )

    # ✅ 對正「結束」後才啟動軌跡規劃與噴灑（相對於對正完成的延遲）
    start_pipeline_after_align = RegisterEventHandler(
        OnProcessExit(
            target_action=alignment,
            on_exit=[
                LogInfo(msg='🎯 Alignment done → starting spray pipeline...'),
                TimerAction(
                    period=3.0,
                    actions=[
                        Node(
                            package='rs485_fire',
                            executable='spray_trajectories_final_node.py',
                            name='spray_planner',
                            output='screen'
                        )
                    ]
                ),
                TimerAction(
                    period=12.0,
                    actions=[
                        Node(
                            package='rs485_fire',
                            executable='spray_executor_node.py',
                            name='spray_executor',
                            output='screen'
                        )
                    ]
                ),
            ]
        )
    )

    # ✅ 用 ExecuteProcess 啟動 shutdown_trigger_node（避免 required）
    shutdown_trigger = ExecuteProcess(
        cmd=['ros2', 'run', 'rs485_fire', 'shutdown_trigger_node.py'],
        name='shutdown_trigger_node',
        output='screen',
        shell=False
    )

    # 🔚 shutdown_trigger_node 結束時，關閉本 launch（不影響上層）
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
        alignment,                    # 先跑對正
        start_pipeline_after_align,   # 對正結束後才啟動後續
        shutdown_trigger,
        shutdown_handler
    ])
