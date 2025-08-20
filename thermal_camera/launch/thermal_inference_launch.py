from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='thermal_camera',
            executable='thermal_camera_node',
            name='thermal_camera',
            output='screen'
        ),
        TimerAction(
            period=3.0,  # 等待熱像儀啟動 3 秒後再執行 triangulate node
            actions=[
                Node(
                    package='thermal_camera',
                    executable='triangulate_node.py',
                    name='triangulate_node',
                    output='screen'
                )
            ]
        ),
        Node(
            package='thermal_camera',
            executable='thermal_fusion_node.py',
            name='thermal_fusion_node',
            output='screen'
        )
    ])
