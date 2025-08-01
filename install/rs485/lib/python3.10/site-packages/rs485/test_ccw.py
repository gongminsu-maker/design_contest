# 파일명: send_motor_command.py

import rclpy
from rclpy.node import Node
import serial
import time
import threading

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
        # 헤더 & ID
        self.header = [0xFF, 0xFE]
        self.motor_id = 0x00
        # 피드백 요청 timer
        self.timer = self.create_timer(1.0,self.request_feedback)
        # 백그라운드 수신 쓰레드 시작
        self.receiver_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.receiver_thread.start()
        # 모터 제어 명령 전송
        self.send_motor_command()

    def send_motor_command(self):
        # 프로토콜 구성
        data_size = 0x06
        speed = [0x00,0xFF] # 19rpm = 0XBE
        mode = 0x03
        direction = 0x01  # CCW: 00, CW: 01 
        time_to_reach = 0x14  # 1.0초
        checksum = (~(self.motor_id + data_size + mode + direction + speed[0] + speed[1] + time_to_reach)) & 0xFF

        data = [checksum]+ [mode, direction, speed[0],speed[1],time_to_reach]
        full_packet = self.header + [self.motor_id] + [data_size] + data

        # 전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"Sent packet: {[hex(b) for b in full_packet]}")

        
    def request_feedback(self):
        data_size = 0x02
        mode = 0xA2 # 속도 피드백
        checksum = (~(self.motor_id + data_size + mode)) & 0xFF
        data = [checksum] + [mode]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] {[ hex(b) for b in full_packet]}")

    def receive_loop(self):
        while True:
            response = self.ser.read(12)
            if response and len(response) >= 12:
                speed = ((response[7] << 8) | response[8]) * 0.1 # 0.1 [rpm]
                position = ((response[9] << 8) | response[10]) * 0.1 #0.1 [degree]
                current = response[11] * 0.1 #100mA , 0.1A
                self.get_logger().info(f"[RX] Speed: {speed:.1f} RPM, Pos: {position:.1f}°, Current: {current:.1f} A")
            time.sleep(0.01)  # CPU 낭비 방지용 짧은 대기


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("keyboardinterrupt!!")
    finally:
        node.ser.close() #시리얼 닫기
        node.destroy_node() #노드 닫기
        rclpy.shutdown()

if __name__ == '__main__':
    main()
