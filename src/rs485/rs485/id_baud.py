# 파일명: find_motor_settings_node.py

import rclpy
from rclpy.node import Node
import serial
import time

class MotorFinderNode(Node):
    def __init__(self):
        super().__init__('motor_finder_node')

        self.declare_parameter('port', '/dev/ttyUSB0')  # 포트 파라미터 선언
        self.serial_port = self.get_parameter('port').get_parameter_value().string_value

        self.baudrates = [9600, 19200, 38400, 57600, 115200]
        self.id_range = range(0, 255)

        self.find_motor_settings()

    def build_ping_packet(self, motor_id):
        header = [0xFF, 0xFE]
        data_size = 0x02
        mode = 0x01  # PING
        packet = [motor_id, data_size, mode]
        checksum = (~sum(packet)) & 0xFF
        return bytearray(header + packet + [checksum])

    def find_motor_settings(self):
        for baud in self.baudrates:
            try:
                ser = serial.Serial(self.serial_port, baudrate=baud, timeout=0.1)
                self.get_logger().info(f"Trying baudrate {baud}...")
                for motor_id in self.id_range:
                    packet = self.build_ping_packet(motor_id)
                    ser.write(packet)
                    time.sleep(0.1)
                    response = ser.read_all()
                    if response:
                        self.get_logger().info(
                            f"✅ Response from ID {motor_id} at baudrate {baud}: {response.hex()}"
                        )
                ser.close()
            except serial.SerialException:
                self.get_logger().warn(f"❌ Failed to open port at baudrate {baud}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorFinderNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
