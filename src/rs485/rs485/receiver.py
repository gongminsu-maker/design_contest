import rclpy as rp
from rclpy.node import Node
import serial
import time
import threading

from geometry_msgs.msg import Twist

class Receiver(Node):
    def __init__(self):
        super().__init__('motorcontroller_node')

        # serial setting
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',         # 적절한 포트로 수정 (ls /dev/ttyUSB*을 통해 포트번호 확인하기)
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        # 파라미터 선언
        self.declare_parameter('mode',0xA0)
        #A0(ping), A1(위치 피드백), A2(속도 피드백), A3(위치제어기 피드백), A4(속도제어기 피드백), A5(통신 응답시간 피드백)
        #A6(외부 감속비 피드백), A7(제어 ON/OFF피드백), A8(위치 제어모드 피드백), A9(절대엔코더 피드백), CD(펌웨어 버전 피드백)
        self.declare_parameter('receiver',"ping") 
        #A0(ping), A1(pos_feedback), A2(vel_feedback), A3(pos_pid_feedback), A4(vel_pid_feedback), A5(tel)
        #A6(reducer), A7(제어 ON/OFF피드백), A8(pos_mode), A9(abs_encoder), CD(펌웨어 버전 피드백)
        

        self.mode = self.get_parameter('mode').get_parameter_value().integer_value
        self.receiver = self.get_parameter('receiver').get_parameter_value().string_value
        #def를 변수로 받고 싶으면 string으로 우선 받고 getattr()로 실제 메서드로 변환
        receiver_func = getattr(self, self.receiver)
        # 헤더 & ID
        self.header = [0xFF, 0xFE]
        self.motor_id = 0x00
        # 피드백 요청 timer
        self.timer = self.create_timer(1.0, self.request_feedback)
        # 백그라운드 수신 쓰레드 시작
        self.receiver_thread = threading.Thread(target=receiver_func, daemon=True)
        self.receiver_thread.start()

    def request_feedback(self):
        data_size = 0x02      # 위치피드백(A1),속도 피드백(A2),위치제어기 피드백(A3), 속도제어기 피드백(A4)
        checksum = (~(self.motor_id + data_size + self.mode)) & 0xFF
        data = [checksum] + [self.mode]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] {[ hex(b) for b in full_packet]}")

    def ping(self):
        while True:
            response = self.ser.read(6)
            if response and len(response) >= 6:
                header = response[0:2]
                id_byte = response[2]
                data_size = response[3]
                ping = response[5]

                # 데이터 정렬을 위한 문법 부분(노이즈, 정렬문제 개선)
                if header == b'\xFF\xFE' and data_size == 0x02 and ping == 0xD0:
                    self.get_logger().info(f"[RX] Valid Ping from ID {id_byte}, Mode: {hex(ping)}")
                else:
                    self.get_logger().warn(f"[RX] Unexpected Ping: {[hex(b) for b in response]}")
            else:
                self.get_logger().warn("[RX] Ping: Incomplete response")
            
            time.sleep(0.01)  # CPU 과다 사용 방지

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
                    self.get_logger().info(f"[RX] Speed: {speed:.1f}°, Pos: {position:.1f} RPM, Current: {current:.1f} A")
                else:
                    self.get_logger().warn(f"[RX] Unexpected Ping")
            else:
                self.get_logger().warn("[RX] Ping: Incomplete response")        
            time.sleep(0.01)  # CPU 낭비 방지용 짧은 대기

    def vel_feedback(self):
        while True:
            response = self.ser.read(12)
            if response and len(response) >= 12:
                if response[6] == 0:
                    direction = "CCW"
                else:
                    direction = "CW"

                speed = ((response[7] << 8) | response[8]) * 0.1     # 0.1 [rpm]
                position = ((response[9] << 8) | response[10]) * 0.1 #0.1 [degree]
                current = response[11] * 0.1 #100mA , 0.1A
                self.get_logger().info(f"[RX] direction: {direction} Speed: {speed:.1f} RPM, Pos: {position:.1f}°, Current: {current:.1f} A")
            time.sleep(0.01)  # CPU 낭비 방지용 짧은 대기
    
    def pos_pid_feedback(self):
        while True:
            response = self.ser.read(10)
            if response and len(response) >= 10:
                kp = response[6]
                ki = response[7]
                kd = response[8]

                self.get_logger().info(f"[RX] kp: {kp:.1f}, ki: {ki:.1f}, kd: {kd:.1f} ")
            time.sleep(0.01)  # CPU 낭비 방지용 짧은 대기


    def Vel_pid_feedback(self):
        while True:
            response = self.ser.read(10)
            if response and len(response) >= 10:
                kp = response[6]
                ki = response[7]
                kd = response[8]

                self.get_logger().info(f"[RX] kp: {kp:.1f}, ki: {ki:.1f}, kd: {kd:.1f} ")
            time.sleep(0.01)  # CPU 낭비 방지용 짧은 대기
    def tel(self):
        while True:
            response = self.ser.read(7)
            if response and len(response) >= 7:
                header = response[0:2]
                id_byte= response[2]
                data_size = response[3]
                mode = response[5]
                time_response = response[6]*100    # 스탭 단위 100us
                if header == b'\xFF\xFE' and data_size == 0x03 and mode == 0xD5:
                    self.get_logger().info(f"[RX] {id_byte}의 통신 응답시간은 {time_response:.1f}us")
                else:
                    self.get_logger().warn(f"[RX] Unexpected response")
            else:
                self.get_logger().warn("[RX] Ping: Incomplete response")        
            time.sleep(0.01)  # CPU 낭비 방지용 짧은 대기

    def reducer(self):
        while True:
            response = self.ser.read(8)
            if response and len(response) >= 8:
                header = response[0:2]
                id_byte = response[2]
                data_size = response[3]
                mode = response[5]
                redu_MSB = response[6]
                redu_LSB = response[7]
                redu = ((redu_MSB << 8) | redu_LSB) * 0.1 # 스탭 단위 0.1ratio
                if header == b'\xFF\xFE' and data_size == 0x04 and mode == 0xD6:
                    self.get_logger().info(f"[RX] {id_byte}의 외부 감속비는 {redu:.1f} : 1")
                else:
                    self.get_logger().warn(f"[RX] Unexpected response")
            else:
                self.get_logger().warn("[RX Incomplete response")
            time.sleep(0.01)

    def pos_mode(self):
        while True:
            response = self.ser.read(7)
            if response and len(response) >= 7:
                header = response[0:2]
                id_byte = response[2]
                data_size = response[3]
                mode = response[5]
                if response[6] == 0:
                    pose_mode = "절대 위치"
                else:
                    pose_mode = "상대 위치"
                if header == b'\xFF\xFE' and data_size == 0x03 and mode == 0xD8:
                    self.get_logger().info(f"[RX] {id_byte}는 {pose_mode}제어 모드")
                else: 
                    self.get_logger().warn(f"[RX] Unexpected response")
            else:
                self.get_logger().warn(f"[RX] Incomplete response")
            time.sleep(0.01)

    def abs_encoder(self):
        while True:
            response = self.ser.read(9)
            if response and len(response) >= 9:
                header = response[0:2]
                id_byte = response[2]
                data_size = response[3]
                mode = response[5]
                if response[6] == 0:
                    direction = "CCW"
                else:
                    direction = "CW"
                en_high = response[7]
                en_low = response[8]
                encoder = ((en_high << 8) | en_low) * 0.01 # 스탭 단위 0.01 degree
                if header == b'\xFF\xFE' and data_size == 0x05 and mode == 0xD9:
                    self.get_logger().info(f"[RX] {id_byte}의 절대 엔코드는 {direction} 방향 위치 {encoder}°")
                else: 
                    self.get_logger().warn(f"[RX] Unexpected response")
            else:
                self.get_logger().warn(f"[RX] Incomplete response")
            time.sleep(0.01)
    



def main(args=None):
    rp.init(args=args)
    node = Receiver()
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


