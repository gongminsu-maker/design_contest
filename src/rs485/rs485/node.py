import math as m
import rclpy as rp
from rclpy.node import Node
import serial
import time
import threading

from geometry_msgs.msg import Twist

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motorcontroller_node')
        # 키보드 속도 명령 수신
        self.sub = self.create_subscription(Twist,'cmd_vel',self.motor_control,10)

        
    def motor_control(self,msg):
        lin_v = float(msg.linear.x)
        ang_v = float(msg.angular.z)

        if lin_v != 0:
            if lin_v > 0:
                self.direction_R = 0x01
                self.direction_L = 0x00
            elif lin_v < 0:
                self.direction_R = 0x00
                self.direction_L = 0x01
            else:
                self.direction_R = 0x00
                self.direction_L = 0x01

        if ang_v != 0:
            if ang_v  >0 :
                self.direction_R = 0x01
                self.direction_L = 0x01
            elif ang_v < 0:
                self.direction_R = 0x00
                self.direction_L = 0x00
            else:
                self.direction_R = 0x00
                self.direction_L = 0x01

        self.get_logger().info(f"{self.direction_R},{self.direction_L}")








def main(args=None):
    rp.init(args=args)
    node = MotorControllerNode()
    try:
        rp.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("keyboardinterrupt!!")
    finally:
        node.ser_L.close()    #시리얼 닫기
        node.ser_R.close()
        node.destroy_node() #노드 닫기
        rp.shutdown()

if __name__ == '__main__':
    main()


