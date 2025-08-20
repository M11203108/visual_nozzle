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
        Node(
            package='thermal_camera',
            executable='thermal_to_rgb_pixel_node.py',
            name='thermal_to_rgb_pixel_node',
            output='screen'
        ),
        TimerAction(
            period=3.0,  # 等待熱像儀啟動 3 秒後再執行 
            actions=[
                Node(
                    package='thermal_camera',
                    executable='fire_detection.py',
                    name='fire_detection',
                    output='screen'
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_tf_pub',
                    arguments=['0.48714', '0.1385', '0.975', '0', '0', '0', 'base_link', 'camera_link']
                )
            ]
        )
    ])
