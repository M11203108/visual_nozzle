from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='thermal_camera',
            executable='thermal_camera_node',
            name='thermal_camera',
            output='screen'
        ),
        # --- 2. thermal_to_rgb_pixel_node --------------------------------------
        Node(
            package='thermal_camera',
            executable='thermal_to_rgb_pixel_node.py',
            name='thermal_to_rgb_pixel_node',
            output='screen'
        )
        
    ])
