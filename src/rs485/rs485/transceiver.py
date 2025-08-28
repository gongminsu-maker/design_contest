import rclpy as rp
from rclpy.node import Node
import serial

class Transceiver(Node):
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
        self.declare_parameter('mode',0x03)
        #01(위치, 속도제어), 02(가감속 위치제어), 03(가감속 속도제어), 04(위치 제어기 설정), 05(속도 제어기 설정)
        #06(ID설정), 07(통신 속도 설정), 08(통신 응답시간 설정), 09(외부 감속비 설정), 0A(제어 on/off 설정), 0B(위치제어 모드 설정)
        #0C(위치 초기화), #0D(공장 초기화)
        self.declare_parameter('transceiver',"acdc_speed") 
        #01(위치 피드백), 02(가감속 위치제어), 03(가감속 속도제어), 04(위치 제어기 설정), 05(속도 제어기 설정)
        #06(ID설정), 07(통신 속도 설정), 08(통신 응답시간 설정), 09(외부 감속비 설정), 0A(제어 on/off 설정), 0B(위치제어 모드 설정)
        #0C(위치 초기화), #0D(공장 초기화)

        self.mode = self.get_parameter('mode').get_parameter_value().integer_value
        self.transceiver = self.get_parameter('transceiver').get_parameter_value().string_value

        # 헤더 & ID
        self.header = [0xFF, 0xFE]
        self.motor_id = 0x00
        #함수 호출
        try:
            #def를 변수로 받고 싶으면 string으로 우선 받고 getattr()로 실제 메서드로 변환
            transceiver_func = getattr(self, self.transceiver)
        except AttributeError:
            self.get_logger().warn("해당 함수는 존재하지 않습니다.")
            return
        transceiver_func()

    #0C 위치 초기화
    def init_pose(self):
        data_size = 0x02      
        checksum = (~(self.motor_id + data_size + self.mode)) & 0xFF
        mode = 0x0C
        data = [checksum] + [mode]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] init_pose  {[ hex(b) for b in full_packet]}")

    #0B 위치제어 모드 설정
    def pose_mode(self):
        data_size = 0x03
        pose_mode = 0x00 # 0x00 절대위치, 0x01 상대위치      
        checksum = (~(self.motor_id + data_size + self.mode + pose_mode)) & 0xFF
        data = [checksum , self.mode , pose_mode]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] 위치 제어 모드 {[ hex(b) for b in full_packet]}")

    
    #01 위치 속도제어 ( 가속도 설정 없음 )
    def pose_vel(self):
        self.init_pose()
        data_size = 0x07      
        direction = 0x00 # CCW(00), CW(01)
        pose_degree= 0x0020 # 0x2328 = 90, 0x4650 = 180, 0x8CA0 = 360 스탭단위 0.01 degree
        MSB_p = (pose_degree >> 8) & 0xFF
        LSB_p = pose_degree & 0xFF
        speed = 0x0032 # 0x0032 = 5RPM , 스탭 단위 0.1 rpm
        MSB_v = (speed >> 8) & 0xFF
        LSB_v = speed & 0xFF
        checksum = (~(self.motor_id + data_size + self.mode + direction + MSB_p + LSB_p + MSB_v + LSB_v )) & 0xFF
        data = [checksum, self.mode, direction , MSB_p , LSB_p , MSB_v , LSB_v]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] {[ hex(b) for b in full_packet]}")
    
    #02 가감속 위치제어
    def acdc_pose(self):
        data_size = 0x06
        direction = 0x00 # CCW(00), CW(01)
        pose_degree= 0x8CA0 # 0x2328 = 90, 0x4650 = 180, 0x8CA0 = 360 스탭단위 0.01 degree
        MSB_p = (pose_degree >> 8) & 0xFF
        LSB_p = pose_degree & 0xFF
        time_to_reach = 0x32 # 스탭 단위 0.1s
        checksum = (~(self.motor_id + data_size + self.mode + direction + MSB_p + LSB_p + time_to_reach)) & 0xFF
        data = [checksum, self.mode, direction, MSB_p, LSB_p, time_to_reach]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] 가감속 위치제어 {[ hex(b) for b in full_packet]}")

    #03 가감속 속도제어
    def acdc_speed(self):
        data_size = 0x06   
        direction = 0x00 #CCW(00), CW(01)
        speed = 0x0000
        MSB_v = (speed >> 8 ) & 0xFF
        LSB_v = speed & 0xFF
        time_to_reach = 0x0A # 스탭 간격 0.1s   
        checksum = (~(self.motor_id + data_size + self.mode + direction + MSB_v + LSB_v + time_to_reach)) & 0xFF
        data = [checksum, self.mode, direction, MSB_v, LSB_v, time_to_reach]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] {[ hex(b) for b in full_packet]}")

    #04 위치 제어기 설정(정격 전류 설정도 가능함)
    def pose_pid(self):
        data_size = 0x06 
        kp = 0x0A
        ki = 0x00
        kd = 0x00
        A = 0x20     
        checksum = (~(self.motor_id + data_size + self.mode + kp + ki + kd + A)) & 0xFF
        data = [checksum, self.mode, kp, ki, kd, A]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] 위치 제어기 설정 {[ hex(b) for b in full_packet]}")

    #05 속도 제어기 설정
    def velocity_pid(self):

        data_size = 0x06 
        kp = 0x0A
        ki = 0x00
        kd = 0x00
        A = 0x20     
        checksum = (~(self.motor_id + data_size + self.mode + kp + ki + kd + A)) & 0xFF
        data = [checksum, self.mode, kp, ki, kd, A]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] 속도 제어기 설정 {[ hex(b) for b in full_packet]}")

    #06 ID 설정
    def id(self):
        data_size = 0x03
        id = 0x00      
        checksum = (~(self.motor_id + data_size + self.mode + id)) & 0xFF
        data = [checksum, self.mode, id]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] id 설정 {[ hex(b) for b in full_packet]}")

    #07 통신 속도 설정
    def tel_velocity(self):
        data_size = 0x03
        baudrate = 0x06 # 초기값 0x06 (9600), 부록 참고      
        checksum = (~(self.motor_id + data_size + self.mode + baudrate)) & 0xFF
        data = [checksum, self.mode, baudrate]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] 보드레이트 {[ hex(b) for b in full_packet]}")

    #08 통신 응답시간 설정
    def tel_response(self):
        data_size = 0x03
        baudrate = 0x06 # 초기값 0x06 (9600), 부록 참고      
        checksum = (~(self.motor_id + data_size + self.mode + baudrate)) & 0xFF
        data = [checksum, self.mode, baudrate]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] 보드레이트 {[ hex(b) for b in full_packet]}")

    #09 외부 감속비 설정
    def reducer(self):
        data_size = 0x04
        i = 0x000A # 초기값 (0x000A) , 스탭단위 0.1 ratio  
        MSB_i = (i >> 8) & 0xFF
        LSB_i = i & 0xFF     
        checksum = (~(self.motor_id + data_size + self.mode + MSB_i + LSB_i )) & 0xFF
        data = [checksum, self.mode, MSB_i, LSB_i] 
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] 외부 감속기 {[ hex(b) for b in full_packet]}")

    #0A 제어 on/off 설정
    def onoff(self):
        data_size = 0x03
        onoff = 0x00 # 초기값(0x00), 0x00 (on), 0x01 (0ff)
        checksum = (~(self.motor_id + data_size + self.mode + onoff)) & 0xFF
        data = [checksum, self.mode, onoff]
        full_packet = self.header + [self.motor_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] on/off {[ hex(b) for b in full_packet]}")



    #0D 공장 초기화
    def init_factory(self):
        data_size = 0x02
        init_fact = 0x0D 
        broad_id = 0xFF     
        checksum = (~(broad_id + data_size + init_fact)) & 0xFF
        data = [checksum, init_fact]
        full_packet = self.header + [broad_id, data_size] + data

        #전송
        self.ser.write(bytearray(full_packet))
        self.get_logger().info(f"[TX] 공장 초기화 {[ hex(b) for b in full_packet]}")
    




def main(args=None):
    rp.init(args=args)
    node = Transceiver()
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



