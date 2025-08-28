import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist
import erial


class Tss(Node):
    def __init__(self):
        super().__init__("zmp_controller")

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



        self.create_publisher(Twist,"/revised/cmd_vel",10)
        time_period = 0.5 # 2HZ
        self.create_timer(time_period, self.re_cmd_vel)










def main(args=None):
    rp.init(args=args)
    node = Tss()
    try:
        rp.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rp.shutdown()



if __name__ =="__main__":
    main()