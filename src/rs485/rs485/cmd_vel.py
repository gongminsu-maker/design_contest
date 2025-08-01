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
        # serial setting
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',         # 적절한 포트로 수정 (ls /dev/ttyUSB*을 통해 포트번호 확인하기)
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )

        # 변수 선언
        self.declare_parameter("receiver","speed_feedback")
        self.receiver = self.get_parameter("receiver").get_parameter_value().string_value
        receiver_func = getattr(self, self.receiver)
        # 헤더 & ID
        self.header = [0xFF, 0xFE]
        self.motor_id = 0x00
        # 피드백 요청 timer
        self.timer = self.create_timer(1.0, self.request_feedback)
        # 백그라운드 수신 쓰레드 시작
        self.receiver_thread = threading.Thread(target=receiver_func, daemon=True)
        self.receiver_thread.start()

    def motor_control(self,msg):
        if msg.linear.x > 0:
            self.direction = 0x01 #CW
        else:
            self.direction = 0x00 #CCW
        vel = abs(msg.linear.x)                        # [m/s]이며 cmd_vel을 통해 들어옴 max 0.26m/s
        radius = 0.135                            #[m]이며 실제 로봇 휠 반지름
        rpm = round((vel/radius)*(60/(2*m.pi)),1) # rpm변환 소수점 첫째자리 반올림 (0.1rpm단위이기 때문에)
        speed_int = int(rpm*10)                   # 계산된 rpm을 실제 프로토콜에 맞는 rpm으로 변환 ex(1300rpm을 명령해도 130rpm으로 돈다)
        
        #속도는 프로토콜 상 2byte임, 1byte의 데이터 크기를 가짐으로 상위 byte, 하위 byte로 나눔
        MSB = (speed_int >> 8) & 0xFF # 상위 byte
        LSB = speed_int & 0xFF        # 하위 byte
        self.speed = [MSB, LSB]       #hex()로하면 문자열임으로 패킷 구성시 사칙연산이 불가능하다. 따라서 [MSB,LSB]로한다

        self.send_motor_command()
            
    def send_motor_command(self):
        # 프로토콜 구성
        data_size = 0x06
        mode = 0x03   
        time_to_reach = 0x0A  # 14 = 2.0초 0A = 1.0초 
        checksum = (~(self.motor_id + data_size + mode + self.direction + self.speed[0] + self.speed[1] + time_to_reach)) & 0xFF

        data = [checksum]+ [mode, self.direction, self.speed[0],self.speed[1],time_to_reach]
        full_packet = self.header + [self.motor_id] + [data_size] + data

        # 전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"Sent packet: {[hex(b) for b in full_packet]}")

    def request_feedback(self):
        data_size = 0x02
        mode = 0xA2       
        checksum = (~(self.motor_id + data_size + mode)) & 0xFF
        data = [checksum] + [mode]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] {[ hex(b) for b in full_packet]}")

    def speed_feedback(self):
        while True:
            response = self.ser.read(12)
            if response and len(response) >= 12:
                header = response[0:2]
                data_size = response[3]
                speed = ((response[7] << 8) | response[8]) * 0.1     # 0.1 [rpm]
                position = ((response[9] << 8) | response[10]) * 0.1 # 0.1 [degree]
                current = response[11] * 0.1 #100mA , 0.1A
                if header == b'\xFF\xFE'  and data_size == 0x08:
                    self.get_logger().info(f"[RX] Speed: {speed:.1f} RPM, Pos: {position:.1f}°, Current: {current:.1f} A")
                else:
                    self.get_logger().warn(f"[RX] Unexpected response")
            else:
                self.get_logger().warn("[RX] Ping: Incomplete response")   
            time.sleep(0.01)  # CPU 낭비 방지용 짧은 대기


    def pos_feedback(self):
        while True:
            response = self.ser.read(12)
            if response and len(response) >= 12:
                header = response[0:2]
                data_size = response[3]
                position = ((response[7] << 8) | response[8]) * 0.01     # 스탭 단위 0.01 [degree]
                speed = ((response[9] << 8) | response[10]) * 0.1 #0.1 [rpm]
                current = response[11] * 0.1 #100mA , 0.1A
                if header == b'\xFF\xFE'  and data_size == 0x08:
                    self.get_logger().info(f"[RX] pos: {position:.1f}°, speed: {speed:.1f} RPM, Current: {current:.1f} A")
                else:
                    self.get_logger().warn(f"[RX] Unexpected Ping")
            else:
                self.get_logger().warn("[RX] Ping: Incomplete response")        
            time.sleep(0.01)  # CPU 낭비 방지용 짧은 대기



def main(args=None):
    rp.init(args=args)
    node = MotorControllerNode()
    try:
        rp.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("keyboardinterrupt!!")
    finally:
        node.ser.close()    #시리얼 닫기
        node.destroy_node() #노드 닫기
        rp.shutdown()

if __name__ == '__main__':
    main()


