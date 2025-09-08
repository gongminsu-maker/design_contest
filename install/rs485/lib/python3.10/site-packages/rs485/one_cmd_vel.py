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
        self.sub = self.create_subscription(Twist,"/revised/cmd_vel",self.motor_control,10)
        # 모터 상태 pub
        self.speed_R_RX = 0
        self.pub = self.create_publisher(Twist,"/motor/cmd_vel",10)
        self.timer_state = self.create_timer(0.1, self.motor_state)     # 10HZ
        

        # serial setting
        self.ser_R = serial.Serial(
            port='/dev/ttyUSB0',         # 적절한 포트로 수정 (ls /dev/ttyUSB*을 통해 포트번호 확인하기)
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )


        # 헤더 & ID
        self.header = [0xFF, 0xFE]
        self.motor_id = 0x00
        self.direction_R = 0x00

        # 피드백 요청 timer
        self.timer_request = self.create_timer(0.05, self.request_feedback) #20HZ  motor_state publisher보다 빠르게 설정

        # 백그라운드 수신 쓰레드 시작
        self.receiver_thread_R = threading.Thread(target=self.speed_feedback_R, daemon=True)      
        self.receiver_thread_R.start()


        # 로봇 세팅값
        self.radius = 0.135                            # [m]이며 실제 로봇 휠 반지름
        self.width = 0.5                               # [m] 휠간거리

    def _parse_one_frame(self,buf:bytearray):
        # 수신 packet 형태
        # header[0:1] + id[2] + datasize[3] + data[4:12] 총 12byte의 패킷
        while True:
            if len(buf) < 4:
                return None # _parse_one_frame을 멈춤 buf가 더 쌓이길 기다림
            if buf[0] == self.header[0] and buf[1] == self.header[1]:
                break
            buf.pop(0) # 데이터는 충분히 들어왔는데 헤더가 맞지 않는다면 바이트를 버리면서 동기화
        data_size = buf[3]  # 속도 수신의 경우 data_size가 0x08 = payload가 8byte임을 알 수 있음
        total_len = 2 + 1 + 1+ data_size  # header + id + size + payload(Data)

        if len(buf) < total_len:  # 프레임이 다 들어오지 않았다면 더 대기
            return None
        
        frame = bytes(buf[:total_len]) # 총 12바이트를 반환함
        del buf[:total_len]
        return frame


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

    def speed_feedback_R(self):
        self.ser_R.timeout = 0.05 # in_waiting,1을 해도 바이트가 없을 때 0.05초 기다림
        buf = bytearray() # 바이트 저장공간
        while True:
            try:
                n = max(self.ser_R.in_waiting,1) # os버퍼에서 쌓여있는 바이트를 확인 없으면 최소 1바이트를 읽을 수 있게 timeout동안 기다림

                chunk = self.ser_R.read(n)
                if chunk:
                    buf.extend(chunk)

                while True:
                    frame = self._parse_one_frame(buf)
                    if frame is None:
                        break
                    payload = frame[4:]
                    if len(payload) <8 :
                        self.get_logger().warn("[RX_R] Payload too short; skipping")
                        continue
                    speed_rpm_01 = ((payload[3] << 8) | payload[4] ) * 0.1
                    # pos랑 전류는 필요하면 사용
                    # pos_deg_01 = ((payload[5] << 8) | payload[6]) * 0.1
                    # current_A_01 = payload[7] * 0.1
                    self.speed_R_RX = speed_rpm_01
            except Exception as e:
                self.get_logger().warn(f"[RX_R] Exception: {e}")

            time.sleep(0.005)

        
    def motor_control(self,msg):
        lin_v = float(msg.linear.x)
        ang_v = float(msg.angular.z)
        vel_r = lin_v + ang_v*self.L/2    # 속도 분배
        self.cal_speed_R(vel_r)
        
        #속도는 프로토콜 상 2byte임, 1byte의 데이터 크기를 가짐으로 상위 byte, 하위 byte로 나눔
        MSB_R = (self.speed_int_R >> 8) & 0xFF             # 상위 byte
        LSB_R = self.speed_int_R & 0xFF                    # 하위 byte
        self.speed_R = [MSB_R, LSB_R]                      #hex()로하면 문자열임으로 패킷 구성시 사칙연산이 불가능하다. 따라서 [MSB,LSB]로한다

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

        # 전송
        self.ser_R.write(bytearray(full_packet_R))
        self.get_logger().info(f"Sent packet to Right: {[hex(b) for b in full_packet_R]}")

    def request_feedback(self):
        data_size = 0x02
        mode = 0xA2
        #Right motor
        checksum_R = (~(self.motor_id + data_size + mode)) & 0xFF
        data_R = [checksum_R , mode]
        full_packet_R = self.header + [self.motor_id, data_size] + data_R

        #전송
        self.ser_R.write(bytearray(full_packet_R))
        self.get_logger().info(f"[TX] {[ hex(b) for b in full_packet_R]}")


    def motor_state(self):
        motor = Twist()
        m_vel_R = self.speed_R_RX*(2*m.pi/60)*self.radius  # rpm -> m/s로 변환 
        m_vel_L = 0.0
        self.get_logger().info(f"R_RX{self.speed_R_RX}")
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

        node.ser_R.close()
        node.destroy_node() #노드 닫기
        rp.shutdown()

if __name__ == '__main__':
    main()


