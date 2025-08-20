from launch import LaunchDescription
# from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
        # ExecuteProcess(
        #     cmd=['/home/robot/newjasmine_ws/install/thermal_camera/lib/thermal_camera/fire_detection.py'],
        #     output='screen'
        # )
        
    ])
