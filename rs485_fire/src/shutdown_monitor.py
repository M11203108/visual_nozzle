#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import signal

class ShutdownMonitor(Node):
    def __init__(self):
        super().__init__('shutdown_monitor')
        self.sub = self.create_subscription(String, '/spray_status', self.cb, 10)

    def cb(self, msg):
        if msg.data.strip() == 'done':
            self.get_logger().info("✅ Received 'done'. Shutting down launch system.")
            os.kill(os.getppid(), signal.SIGINT)  # Ctrl+C 效果，整個 launch 關閉

def main():
    rclpy.init()
    node = ShutdownMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
