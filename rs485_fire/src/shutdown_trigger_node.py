#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ShutdownTriggerNode(Node):
    def __init__(self):
        super().__init__('shutdown_trigger_node')
        self.sub = self.create_subscription(String, '/spray_status', self.cb, 10)
        self.shutdown_flag = False

    def cb(self, msg):
        if msg.data.strip() == 'done':
            self.get_logger().info("🟢 任務完成訊號收到，準備結束 launch")
            self.shutdown_flag = True  # 設旗標，但不要在這 sleep()

def main():
    rclpy.init()
    node = ShutdownTriggerNode()
    try:
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node, timeout_sec=0.1)

        # ⏱️ 離開 spin loop，表示旗標被設為 True
        node.get_logger().info("⌛ 等待 5 秒讓其他節點完成釋放後再關閉 ...")
        time.sleep(5)

    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ shutdown_trigger_node exited cleanly")

if __name__ == '__main__':
    main()
