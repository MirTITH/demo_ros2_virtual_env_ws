import torch
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")

        self.get_logger().info(f"PyTorch version: {torch.__version__}")
        self.get_logger().info(f"CUDA available: {torch.cuda.is_available()}")

        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        x = torch.rand(5, 3)
        msg.data = f"Random Tensor: {x} | Count: {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.data}'")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
