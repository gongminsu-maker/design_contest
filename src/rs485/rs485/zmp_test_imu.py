import rclpy as rp
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import TransformStamped, Point
import tf2_ros
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from tf_transformations import euler_from_quaternion
from rclpy.duration import Duration as rclpyDuration   # timeout용
# imu 센서 sub
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class BaseBroad(Node):
    # 실제 IMU센서를 읽어서 지지면이 움직이는 것을 확인하는 코드(준비물: IMU)
    
    def __init__(self):
        super().__init__("Basebroad_node")

        # IMU SUB
        self.imu = self.create_subscription(Imu,'/base/imu/data',self.callback_imu,10)
        # cmd_vel SUB
        self.sub_cmd_vel = self.create_subscription(Twist,"/cmd_vel",self.callback_vel,10)
        # motor state SUB
        self.sub_motor_state = self.create_subscription(Twist,"/motor/cmd_vel", self.callback_motor_state,10)
        
        time_period = 0.5 # 2HZ [모터의 디폴트 도달시간이 0.5초임으로 timer와 맞춤]
        self.create_timer(time_period, self.re_cmd_vel)
        # revised cmd_vel PUB
        self.pub = self.create_publisher(Twist,"/revised/cmd_vel",10)

        # 브로드캐스터
        self.tf = tf2_ros.StaticTransformBroadcaster(self)  # static broadcaster
        self.tf_base = tf2_ros.TransformBroadcaster(self)   # broadcaster

        #Marker용 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # 속성값
        x = -0.1
        y = 0.0
        h = 0.135
        self.CoG_local = np.array([x, y, h])
        self.sf = 5.0 # default = 1.0 
        self.M = 25.0
        self.xu = 0.295
        self.xl = -0.295
        self.yu = 0.309
        self.yl = -0.309
        self.v_max = 0.26
        self.v_min = -0.26
        self.a_max = 0.52  # 0.26m/s를 0.5초에 도달
        self.a_min = -0.52
        self.w_max = 1.04 # (v_r - v_l)/wheel width
        self.w_min = -1.04
        self.alpha_max = 2.08 # 1.04rad/s를 0.5초에 도달
        self.alpha_min = -2.08
        self.circle_ac_max = self.w_max*self.w_max*abs(x)
        self.del_t = 0.5    # re_cmd_vel의 주기와 동일

        # 초기값
        self.prev_vel = 0.0
        self.prev_ang = 0.0
        self.a_drive = 0.0
        self.alpha_drive = 0.0
        self.v_des = 0.0
        self.w_des = 0.0
        self.cmd_timeout = 1.0
        self.last_cmd_time = self.get_clock().now()


        transforms = []
        transforms.append(self.robot_broad("CoG", -0.1, 0.0, 0.135))
        transforms.append(self.robot_broad("robot_FL", 0.297, -0.314, 0.0))
        transforms.append(self.robot_broad("robot_FR", 0.297,  0.314, 0.0))
        transforms.append(self.robot_broad("robot_RR", -0.297, 0.314, 0.0))
        transforms.append(self.robot_broad("robot_RL", -0.297,-0.314, 0.0))
        self.tf.sendTransform(transforms)   # static broad
    
    def callback_motor_state(self,msg):
        self.prev_vel = msg.linear.x
        self.prev_ang = msg.angular.z

    def callback_vel(self,msg):
        self.v_des = float(max(self.v_min, min(self.v_max, msg.linear.x)))
        self.w_des = float(max(self.w_min, min(self.w_max, msg.angular.z)))
        self.last_cmd_time = self.get_clock().now()
    
    def callback_imu(self,msg):
        self.qx = msg.orientation.x
        self.qy = msg.orientation.y
        self.qz = msg.orientation.z
        self.qw = msg.orientation.w
        self.broad_base()

     
    def broad_base(self):                # tilted ground -> base (broadcaster)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = -self.qx
        t.transform.rotation.y = self.qy
        t.transform.rotation.z = self.qz
        t.transform.rotation.w = self.qw

        self.draw_marker("base","robtf", 1, -0.1, 0.0, 0.135, [0., 0., 0., 1.], Marker.SPHERE, [0.0, 1.0, 0.0], 0.1)
        self.draw_rectangle_marker()
        self.draw_zmp_marker()
        self.tf_base.sendTransform(t)

    def robot_broad(self, child, x, y, z):   # base -> support (static broadcaster)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base"
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        return t

    def draw_marker(self, f_id,ns, marker_id, x, y, z, quat, marker_type, rgb, scale, lifetime=0):
        m = Marker()
        m.header.frame_id = f_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns                #namespace
        m.id = marker_id         # id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.x = quat[0]
        m.pose.orientation.y = quat[1]
        m.pose.orientation.z = quat[2]
        m.pose.orientation.w = quat[3]
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color.r = rgb[0]
        m.color.g = rgb[1]
        m.color.b = rgb[2]
        m.color.a = 1.0
        m.lifetime = Duration(sec=lifetime)
        self.marker.publish(m)

    def draw_rectangle_marker(self):
        names = ["robot_FL", "robot_FR", "robot_RR", "robot_RL", "robot_FL"]  # 직사각형 
        points = []
        for name in names:
            try:
                tf = self.tf_buffer.lookup_transform("base", name, rp.time.Time(), timeout=rclpyDuration(seconds=0.2))
                t = tf.transform.translation
                points.append(Point(x=t.x, y=t.y, z=t.z))
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed for {name}: {e}")
                return
        m = Marker()
        m.header.frame_id = "base"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "rectangle"
        m.id = 10
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.points = points
        m.pose.orientation.w = 1.0
        self.marker.publish(m)

    def draw_zmp_marker(self):
        # base frame기준으로 zmp 계산
        g = 9.8
        h = 0.135
        M = 25.0
        v = 0.0    # 최대속도 0.26m/s
        w = 0.0    # 최대각속도 1.04rad/s
        t = 0.5    # 도달시간 디폴드값

        omega = np.array([0.0, 0.0, w])
        alpha = np.array([0.0, 0.0, w / t])
        CoG_a_local = np.array([v / t, 0.0, 0.0])

        # 지면 기울기 반영 world에서 base의 좌표계 회전을 listen해서 th, psi갱신

        try:
            # world를 base기준으로 바라봄
            tf = self.tf_buffer.lookup_transform("base", "world", rp.time.Time(), timeout=rclpyDuration(seconds=0.2))
            rot = tf.transform.rotation
            q = [rot.x, rot.y, rot.z, rot.w]
            R = euler_from_quaternion(q)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed for world → base: {e}")
            return

        psi = R[0]  # Roll
        th = R[1]   # pitch
        yaw = R[2]  # yaw

        R_pitch = np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])
        R_roll = np.array([[1, 0, 0], [0, math.cos(psi), -math.sin(psi)], [0, math.sin(psi), math.cos(psi)]])
        R_yaw = np.array([[math.cos(yaw),-math.sin(yaw), 0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
        R_world_to_local = R_yaw @ R_pitch @ R_roll  # ros회전변환순서 ZYX

        g_world = np.array([0.0, 0.0, g])
        self.g_local = R_world_to_local @ g_world 
         
        # non_force zmp
        self.x_nf = (M * self.g_local[2] * self.CoG_local[0] - M * self.g_local[0] * self.CoG_local[2]) / (M * self.g_local[2])
        self.y_nf = (M * self.g_local[2] * self.CoG_local[1] - M * self.g_local[1] * self.CoG_local[2]) / (M * self.g_local[2])
        #self.get_logger().info(f"g좌표: {self.g_local}")

        # 구심가속도, 회전접선가속도
        a_c_local = np.cross(omega, np.cross(omega, self.CoG_local))
        circle_ac_local = np.cross(alpha,self.CoG_local)
        # 다음 수식은 결과 값임으로 abs(self.g_local)을 사용해야함.
        x_zmp_local = self.x_nf - h * (CoG_a_local[0] + a_c_local[0]) / abs(self.g_local[2])
        y_zmp_local = self.y_nf - h * (CoG_a_local[1] + a_c_local[1] + circle_ac_local[1]) / abs(self.g_local[2])
        zmp_local = np.array([x_zmp_local, y_zmp_local, 0.0])

        self.draw_marker("base","zmp", 200, zmp_local[0], zmp_local[1], 0.0, [0., 0., 0., 1.], Marker.SPHERE, [0.0, 0.0, 1.0], 0.07)

    # 선형 운동시 선형 가속도에 의한 zmp x축
    def lin_accel_bounds(self):

        # 예외처리
        try: 
            g_local = self.g_local
            x_nf = self.x_nf
        except AttributeError:
            g_local = np.array([0.0,0.0,9.8])
            x_nf = self.CoG_local[0]

        # stability index
        # 
        Sau = (1- (g_local[2]*(x_nf - self.xu))/(self.a_max*self.CoG_local[2]))/2
        Sal = (1+ (g_local[2]*(x_nf - self.xl))/(self.a_max*self.CoG_local[2]))/2

        # Sau,Sal이 1일 때가 zmp의 경계가 polygon에 접하는 순간
        # 안전계수(self.sf)를 곱해 커스텀 가능

        if Sau >= 1*self.sf and Sal >= 1*self.sf:
            a_lower = self.a_min
            a_upper = self.a_max
            self.get_logger().warn(f"[선형 안정], Sau: {Sau}, Sal: {Sal}, a_lower: {a_lower}, a_upper: {a_upper}")
        elif Sau >0 and Sau <1*self.sf and Sal >= self.sf:
            a_lower = (2/self.sf)*(0.5*self.sf-min(Sau,1*self.sf))*self.a_max
            a_upper = self.a_max
            self.get_logger().warn(f"[선형 감속 제약], Sau: {Sau}, Sal: {Sal}, a_lower: {a_lower}, a_upper: {a_upper}")
        elif Sau >= 1*self.sf and Sal > 0 and Sal < 1*self.sf :
            a_lower = self.a_min
            a_upper = (2/self.sf)*(min(Sal,1*self.sf)-0.5*self.sf)*self.a_max
            self.get_logger().warn(f"[선형 가속 제약], Sau: {Sau}, Sal: {Sal}, a_lower: {a_lower}, a_upper: {a_upper}")
        elif Sau >0 and Sau < 1*self.sf and Sal > 0 and Sal <1*self.sf:
            a_lower = (2/self.sf)*(0.5*self.sf-min(Sau,1*self.sf))*self.a_max
            a_upper = (2/self.sf)*(min(Sal,1*self.sf)-0.5*self.sf)*self.a_max
            self.get_logger().warn(f"[선형 가감속 제약], Sau: {Sau}, Sal: {Sal}, a_lower: {a_lower}, a_upper: {a_upper}")
        else:
            a_lower = 0.0
            a_upper = 0.0
            self.get_logger().warn(f"[전복], Sau: {Sau}, Sal: {Sal}")
        return a_lower, a_upper, Sau, Sal
    
    # 회전운동시 회전접선가속도에 의한 y축 zmp
    def alpha_bounds(self):

        # 예외처리
        try: 
            g_local = self.g_local
            y_nf = self.y_nf
        except AttributeError:
            g_local = np.array([0.0,0.0,9.8])
            y_nf = self.CoG_local[1]

        # stability index
        Sapu = (1- (g_local[2]*(y_nf - self.yu))/(self.alpha_max*self.CoG_local[2]))/2
        Sapl = (1+ (g_local[2]*(y_nf - self.yl))/(self.alpha_max*self.CoG_local[2]))/2

        if Sapu >= 1*self.sf and Sapl >= 1*self.sf:
            alpha_lower = self.alpha_min
            alpha_upper = self.alpha_max
            self.get_logger().warn(f"[회전 접선 안정], Sapu: {Sapu}, Sapl: {Sapl}, alpha_lower: {alpha_lower}, alpha_upper: {alpha_upper}")
        elif Sapu >0 and Sapu <1*self.sf and Sapl >= 1*self.sf:
            alpha_lower = (2/self.sf)*(0.5*self.sf-min(Sapu,1*self.sf))*self.alpha_max
            alpha_upper = self.alpha_max
            self.get_logger().warn(f"[회전 접선감속 제약], Sapu: {Sapu}, Sapl: {Sapl}, alpha_lower: {alpha_lower}, alpha_upper: {alpha_upper}")
        elif Sapu >= 1*self.sf and Sapl > 0 and Sapl < 1*self.sf :
            alpha_lower = self.alpha_min
            alpha_upper = (2/self.sf)*(min(Sapl,1*self.sf)-0.5*self.sf)*self.alpha_max
            self.get_logger().warn(f"[회전 접선 가속 제약], Sapu: {Sapu}, Sapl: {Sapl}, alpha_lower: {alpha_lower}, alpha_upper: {alpha_upper}")
        elif Sapu >0 and Sapu < 1*self.sf and Sapl > 0 and Sapl <1*self.sf:
            alpha_lower = (2/self.sf)*(0.5*self.sf-min(Sapu,1*self.sf))*self.alpha_max
            alpha_upper = (2/self.sf)*(min(Sapl,1*self.sf)-0.5*self.sf)*self.alpha_max
            self.get_logger().warn(f"[회전 접선 가감속 제약], Sapu: {Sapu}, Sapl: {Sapl}, alpha_lower: {alpha_lower}, alpha_upper: {alpha_upper}")
        else:
            alpha_lower = 0.0
            alpha_upper = 0.0
            self.get_logger().warn(f"[전복], Sapu: {Sapu}, Sapl: {Sapl}")
        return alpha_lower, alpha_upper, Sapu, Sapl
    
    # 회전운동시 구심가속도에 의한 x축 zmp
    def w_bounds(self):

        # 예외처리
        try: 
            g_local = self.g_local
            x_nf = self.x_nf
        except AttributeError:
            g_local = np.array([0.0,0.0,9.8])
            x_nf = self.CoG_local[0]

        # stability index
        Swl = (1+ (abs(g_local[2])*(x_nf - self.xl))/(self.circle_ac_max*self.CoG_local[2]))/2
        Sw = Swl # 구심가속도는 하한에만 영향이 있음


        if Sw >= 1*self.sf: 
            circle_ac = self.circle_ac_max
            self.get_logger().warn(f"[회전 구심 안정], Sw: {Sw},circle_ac: {circle_ac}") 
        elif Sw >0 and Sw <1*self.sf: 
            circle_ac = max(0,(2/self.sf)*(0.5*self.sf-min(Sw,1*self.sf))*self.circle_ac_max)
            self.get_logger().warn(f"[회전 구심 제약], Sw: {Sw},circle_ac: {circle_ac}")
        else:
            circle_ac = 0.0
            self.get_logger().warn(f"[전복], Sw: {Sw}")
        w_lower = max(self.w_min,-math.sqrt(circle_ac/abs(self.CoG_local[0])))
        w_upper = min(self.w_max,math.sqrt(circle_ac/abs(self.CoG_local[0])))
        return w_lower, w_upper, Sw
    
    def re_cmd_vel(self):
        now = self.get_clock().now()
        # 1.0초동안 cmd_vel명령 없을 시 정지
        if (now - self.last_cmd_time).nanoseconds*1e-9 > self.cmd_timeout:
            self.v_des = 0.0
            self.w_des = 0.0

        a_cmd = (self.v_des - self.prev_vel) / self.del_t
        a_lower, a_upper, Sau, Sal = self.lin_accel_bounds()

        self.a_drive = max(a_lower, min(a_upper, a_cmd))

        ugv_lin_vel = self.prev_vel + self.a_drive * self.del_t
        re_cmd_vel = max(self.v_min, min(self.v_max, ugv_lin_vel))

        alpha_cmd = (self.w_des - self.prev_ang) / self.del_t
        alpha_lower, alpha_upper, Sapu, Sapl = self.alpha_bounds()
        w_lower, w_upper, Sw = self.w_bounds()

        self.alpha_drive = max(alpha_lower, min(alpha_upper, alpha_cmd))
        ugv_ang_vel = self.prev_ang + self.alpha_drive * self.del_t
        re_ang_vel = max(w_lower, min(w_upper,ugv_ang_vel)) # alpha와 w의 zmp제한 교집합
        

        v= Twist()
        v.linear.x = re_cmd_vel
        v.angular.z = re_ang_vel
        self.pub.publish(v)

def main(args=None):
    rp.init(args=args)
    node = BaseBroad()
    try:
        rp.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rp.shutdown()

if __name__ == "__main__":
    main()