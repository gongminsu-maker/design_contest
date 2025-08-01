import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdSub(Node):
    def __init__(self):
        super().__init__("cmdsub_node")
        self.sub = self.create_subscription(Twist,"cmd_vel",self.display,10)

    def display(self,msg):
        self.get_logger().info(f"{msg.linear.x}")










def main(args=None):
    rp.init(args=args)
    node = CmdSub()
    try:
        rp.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("keyboardinterrupt!!")
    finally:
        node.destroy_node() #노드 닫기
        rp.shutdown()

if __name__ == '__main__':
    main()
