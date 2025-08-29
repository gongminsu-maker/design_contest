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

        self.L = 0.5 #휠간격
        # 키보드 속도 명령 수신
        self.sub = self.create_subscription(Twist,"/cmd_vel",self.motor_control,10)
        # 모터 상태 pub
        self.pub = self.create_publisher(Twist,"/motor/cmd_vel")
        self.timer = self.create_timer(0.1, self.motor_state)     # 10HZ
        

        # serial setting
        self.ser_R = serial.Serial(
            port='/dev/ttyUSB0',         # 적절한 포트로 수정 (ls /dev/ttyUSB*을 통해 포트번호 확인하기)
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        self.ser_L = serial.Serial(
           port="/dev/ttyUSB1",
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
        self.timer = self.create_timer(0.05, self.request_feedback) #20HZ  motor_state publisher보다 빠르게 설정

        # 백그라운드 수신 쓰레드 시작
        self.receiver_thread_R = threading.Thread(target=self.speed_feedback_R, daemon=True)
        self.receiver_thread_L = threading.Thread(target=self.speed_feedback_L, daemon=True)        
        self.receiver_thread_R.start()
        self.receiver_thread_L.start()

        # 로봇 세팅값
        self.radius = 0.135                            # [m]이며 실제 로봇 휠 반지름
        self.width = 0.5                               # [m] 휠간거리

    def cal_speed_R(self, v):
        rpm = round((v/self.radius)*(60/(2*m.pi)),1)   # rpm변환 소수점 첫째자리 반올림 (0.1rpm단위이기 때문에)
        if v != 0:
            if v > 0:
                self.direction_R = 0x01

            else:
                self.direction_R = 0x00
        else:
            self.direction_R = 0x00

        self.speed_int_R = abs(int(rpm*10))              # 계산된 rpm을 실제 프로토콜에 맞는 rpm으로 변환 ex(1300rpm을 명령하면 130rpm으로 돈다)
        return self.speed_int_R   

    def cal_speed_L(self, v):
       rpm = round((v/self.radius)*(60/(2*m.pi)),1)   # rpm변환 소수점 첫째자리 반올림 (0.1rpm단위이기 때문에)
       if v != 0:
           if v > 0:
               self.direction_L = 0x00

           else:
               self.direction_L = 0x01
       else:
           # self.direction_L = 0x00

        self.speed_int_L = abs(int(rpm*10))              # 계산된 rpm을 실제 프로토콜에 맞는 rpm으로 변환 ex(1300rpm을 명령하면 130rpm으로 돈다)
        return self.speed_int_L                     

        
    def motor_control(self,msg):
        lin_v = float(msg.linear.x)
        ang_v = float(msg.angular.z)
        vel_r = lin_v + ang_v*self.L/2    # 속도 분배
        vel_l = lin_v - ang_v*self.L/2
        self.cal_speed_R(vel_r)
        self.cal_speed_L(vel_l)
        
        #속도는 프로토콜 상 2byte임, 1byte의 데이터 크기를 가짐으로 상위 byte, 하위 byte로 나눔
        MSB_R = (self.speed_int_R >> 8) & 0xFF             # 상위 byte
        LSB_R = self.speed_int_R & 0xFF                    # 하위 byte
        self.speed_R = [MSB_R, LSB_R]                      #hex()로하면 문자열임으로 패킷 구성시 사칙연산이 불가능하다. 따라서 [MSB,LSB]로한다
        MSB_L= (self.speed_int_L >> 8) & 0xFF              # 상위 byte
        LSB_L = self.speed_int_L & 0xFF                    # 하위 byte
        self.speed_L = [MSB_L, LSB_L]                      #hex()로하면 문자열임으로 패킷 구성시 사칙연산이 불가능하다. 따라서 [MSB,LSB]로한다

        self.send_motor_command()
            
    def send_motor_command(self):
        # 프로토콜 구성
        data_size = 0x06
        mode = 0x03   
        time_to_reach = 0x05  # 0x05 = 0.5초
        #Right motor
        checksum_R = (~(self.motor_id + data_size + mode + self.direction_R + self.speed_R[0] + self.speed_R[1] + time_to_reach)) & 0xFF
        data_R = [checksum_R, mode, self.direction_R, self.speed_R[0],self.speed_R[1],time_to_reach]
        full_packet_R = self.header + [self.motor_id, data_size] + data_R  # 리스트 평탄화
        #Left motor
        checksum_L = (~(self.motor_id + data_size + mode + self.direction_L+ self.speed_L[0] + self.speed_L[1] + time_to_reach)) & 0xFF
        data_L = [checksum_L, mode, self.direction_L, self.speed_L[0], self.speed_L[1], time_to_reach]
        full_packet_L = self.header + [self.motor_id , data_size] + data_L  

        # 전송
        self.ser_R.write(bytearray(full_packet_R))
        self.ser_L.write(bytearray(full_packet_L))
        self.get_logger().info(f"Sent packet to Right: {[hex(b) for b in full_packet_R]}")
        self.get_logger().info(f"Sent packet to Left: {[hex(b) for b in full_packet_L]}")

    def request_feedback(self):
        data_size = 0x02
        mode = 0xA2
        #Right motor
        checksum_R = (~(self.motor_id + data_size + mode)) & 0xFF
        data_R = [checksum_R , mode]
        full_packet_R = self.header + [self.motor_id, data_size] + data_R

        #Left motor
        checksum_L = (~(self.motor_id + data_size + mode)) & 0xFF
        data_L = [checksum_L , mode]
        full_packet_L = self.header + [self.motor_id, data_size] +  data_L

        #전송
        self.ser_R.write(bytearray(full_packet_R))
        self.ser_L.write(bytearray(full_packet_L))
        self.get_logger().info(f"[TX] {[ hex(b) for b in full_packet_R]}")

    def speed_feedback_R(self):
        while True:
            response = self.ser_R.read(12)
            if response and len(response) >= 12:
                header = response[0:2]
                data_size = response[3]
                self.speed_R = ((response[7] << 8) | response[8]) * 0.1     # 0.1 [rpm]
                position = ((response[9] << 8) | response[10]) * 0.1 # 0.1 [degree]
                current = response[11] * 0.1 #100mA , 0.1A
                if header == b'\xFF\xFE'  and data_size == 0x08:
                    self.get_logger().info(f"[RX_R] Speed: {self.speed_R:.1f} RPM, Pos: {position:.1f}°, Current: {current:.1f} A")
                else:
                    self.get_logger().warn(f"[RX_R] Unexpected response")
            else:
                self.get_logger().warn("[RX_R] Ping: Incomplete response")   
            time.sleep(0.01)  # CPU 낭비 방지용 짧은 대기

    def speed_feedback_L(self):
        while True:
            response = self.ser_L.read(12)
            if response and len(response) >= 12:
                header = response[0:2]
                data_size = response[3]
                self.speed_L = ((response[7] << 8) | response[8]) * 0.1     # 0.1 [rpm]
                position = ((response[9] << 8) | response[10]) * 0.1 # 0.1 [degree]
                current = response[11] * 0.1 #100mA , 0.1A
                if header == b'\xFF\xFE'  and data_size == 0x08:
                    self.get_logger().info(f"[RX_L] Speed: {self.speed_L:.1f} RPM, Pos: {position:.1f}°, Current: {current:.1f} A")
                else:
                    self.get_logger().warn(f"[RX_L] Unexpected response")
            else:
                self.get_logger().warn("[RX_L] Ping: Incomplete response")   
            time.sleep(0.01)  # CPU 낭비 방지용 짧은 대기

    def motor_state(self):
        motor = Twist()
        m_vel_R = self.speed_R*(2*m.pi/60)*self.radius  # rpm -> m/s로 변환 
        m_vel_L = self.speed_L*(2*m.pi/60)*self.radius
        m_vel = (m_vel_R + m_vel_L)/2    # 선속도
        m_ang = (m_vel_R - m_vel_L)/(self.width)
        motor.linear.x = m_vel
        motor.angular.z = m_ang
        self.pub.publish(motor)






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


