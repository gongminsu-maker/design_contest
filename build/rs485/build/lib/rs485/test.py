# 파일명: send_motor_command.py

import rclpy
from rclpy.node import Node
import serial
import time

class MotorCommandNode(Node):
    def __init__(self):
        super().__init__('motor_command_node')

        # ★ 시리얼 포트 설정 (수정 필요)
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',  # 적절한 포트로 수정 (ex. '/dev/ttyUSB0')
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )

        # 모터 제어 명령 전송
        self.send_motor_command()

    def send_motor_command(self):
        # 프로토콜 구성
        header = [0xFF, 0xFE]
        motor_id = 0x00
        data_size = 0x06
        speed = [0x05,0x14]
        mode = 0x03
        direction = 0x01  # CCW1300 RPM
        time_to_reach = 0x0A  # 1.0초

        packet = [mode, direction] + speed + [time_to_reach]
        checksum =  0xD3 # 1바이트만 남기기

        full_packet = header + [motor_id] + [data_size] + [checksum] + packet

        # 전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"Sent packet: {[hex(b) for b in full_packet]}")

        # 닫기 (지속제어라면 주석처리)
        self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
