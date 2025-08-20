import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        

        self.subscriptions = self.create_subscription(
            String,
            "/fire_info",
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
         
    def listener_callback(self, msg):

        self.get_logger().info(f'Received message: "{msg.data}"')
        rclpy.shutdown()




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()