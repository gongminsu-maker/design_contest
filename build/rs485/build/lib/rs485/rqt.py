import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

class SineCosinePublisher(Node):
    def __init__(self):
        super().__init__('sine_cosine_publisher')

        # 퍼블리셔 2개 (sin, cos)
        self.pub_sin = self.create_publisher(Float32, 'sine', 10)
        self.pub_cos = self.create_publisher(Float32, 'cosine', 10)

        # 타이머 (0.1초마다 실행)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start_time = time.time()

    def timer_callback(self):
        t = time.time() - self.start_time
        msg_sin = Float32()
        msg_cos = Float32()

        msg_sin.data = math.sin(t)
        msg_cos.data = math.cos(t)

        self.pub_sin.publish(msg_sin)
        self.pub_cos.publish(msg_cos)

        self.get_logger().info(f"sin: {msg_sin.data:.3f}, cos: {msg_cos.data:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = SineCosinePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
