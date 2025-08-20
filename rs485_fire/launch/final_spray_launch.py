# rs485_fire/launch/final_spray_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    planner = Node(
        package   = 'rs485_fire',
        executable= 'spray_trajectories_final_node.py',   
        name      = 'spray_planner',
        output    = 'screen'
    )

    executor = Node(
        package   = 'rs485_fire',
        executable= 'spray_executor_node.py',        
        name      = 'spray_executor',
        output    = 'screen'
    )

    return LaunchDescription([
        planner,
        # 3 秒後再啟動執行節點（可視需要調整）
        TimerAction(period=3.0, actions=[executor])
    ])
