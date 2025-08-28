import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import numpy as np


class Tss(Node):
    def __init__(self):
        super().__init__("zmp_controller")
        # teleop 키보드로 입력해가면서 TSS가 잘 작동하는지 확인하는 코드(준비물: 모터, cmd_vel.py)

        self.sub_cmd_vel = self.create_subscription(Twist,"/cmd_vel",self.callback_vel,10)   # 컨트롤러 속도 명령 구독
        self.sub_motor_state = self.create_subscription(Twist,"/motor/cmd_vel", self.callback_motor_state,10) # 모터 속도 상태 피드백
        self.pub = self.create_publisher(Twist,"/revised/cmd_vel",10)                # 수정된 속도 명령
        time_period = 0.5 # 2HZ
        self.create_timer(time_period, self.re_cmd_vel)

        # 세팅값
        self.M = 25
        self.xu = 0.295
        self.xl = -0.295
        self.v_max = 0.26
        self.v_min = -0.26
        self.a_max = 0.52
        self.a_min = -0.52
        self.del_t = 0.5
        self.cog = [-0.05, 0 ,0.135]
        self.g = 9.8

    def callback_motor_state(self,msg):
        self.prev_vel = msg.linear.x
        self.prev_ang = msg.angular.z


    def callback_vel(self,msg):
        v_cmd = msg.linear.x  # 우선 직진, 후진 테스트만
        a_cmd = (v_cmd-self.prev_vel)/self.del_t

        th = math.radians(-45) # uphill
        yaw = math.radians(0)
        psi = math.radians(0)

        R_pitch = np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])
        R_roll = np.array([[1, 0, 0], [0, math.cos(psi), -math.sin(psi)], [0, math.sin(psi), math.cos(psi)]])
        R_yaw = np.array([[math.cos(yaw),-math.sin(yaw), 0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
        R_world_to_local = R_yaw @ R_pitch @ R_roll  # ros회전변환순서 ZYX

        g_world = np.array([0.0, 0.0, self.g])
        g_local = R_world_to_local @ g_world 
        # non_force zmp
        x_nf = (self.M * g_local[2] * self.cog[0] - self.M * g_local[0] * [2]) / (self.M * g_local[2])
        # stability index
        Sau = (1- (g_local[2]*(x_nf - self.xu))/(self.a_max*self.cog[2]))/2
        Sal = (1+( g_local[2]*(x_nf - self.xl))/(self.a_max*self.cog[2]))/2

        if Sau >= 1 and Sal >= 1:
            a_lower = self.a_min
            a_upper = self.a_max
        elif Sau >0 and Sau <1 and Sal >=1 :
            a_lower = 2*(0.5-min(Sau,1))*self.a_max
            a_upper = self.a_max
        elif Sau >=1 and Sal > 0 and Sal <1 :
            a_lower = self.a_min
            a_upper = 2*(min(Sal,1)-0.5)*self.a_max
        elif Sau >0 and Sau <1 and Sal > 0 and Sal <1:
            a_lower = 2*(0.5-min(Sau,1))*self.a_max
            a_upper = 2*(min(Sal,1)-0.5)*self.a_max
        else:
            a_lower = -0.01
            a_upper = 0.01
            self.get_logger().warn(f"전복, Sau: {Sau}, Sal: {Sal}")
        self.a_drive = max(a_lower, min(a_upper,a_cmd))


    def re_cmd_vel(self):
        v = Twist()
        ugv_lin_vel = self.prev_vel + self.a_drive*self.del_t
        re_cmd_vel = max(self.v_min , min(self.v_max, ugv_lin_vel))
        v.linear.x = re_cmd_vel
        self.pub.publish(v)


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