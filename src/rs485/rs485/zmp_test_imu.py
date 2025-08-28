import rclpy as rp
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import TransformStamped, Point
import tf2_ros
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_quaternion
from rclpy.duration import Duration as rclpyDuration   # timeout용
# imu 센서 sub
from sensor_msgs.msg import Imu

class BaseBroad(Node):
    # 실제 IMU센서를 읽어서 지지면이 움직이는 것을 확인하는 코드(준비물: IMU)
    
    def __init__(self):
        super().__init__("Basebroad_node")

        # IMU SUB
        self.imu = self.create_subscription(Imu,"/imu/data",self.callback_imu,10)

        # 브로드캐스터
        self.tf = tf2_ros.StaticTransformBroadcaster(self)  # static broadcaster
        self.tf_base = tf2_ros.TransformBroadcaster(self)   # broadcaster

        #Marker용 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker = self.create_publisher(Marker, 'visualization_marker', 10)

        self.theta_tilted = 0.0                     # world 기준 tilted 각도 
        self.tilted_direct = -1
        self.theta_base = 0.0                       # tilted 기준 base각도
        self.base_direct = -1
        self.theta_max = math.radians(56)    #  pi/4
        self.theta_min = -math.radians(56)   # -pi/4

        self.timer = self.create_timer(0.15, self.broad_tilted_ground)

        transforms = []
        transforms.append(self.robot_broad("CoG", -0.1, 0.0, 0.135))
        transforms.append(self.robot_broad("robot_FL", 0.297, -0.314, 0.0))
        transforms.append(self.robot_broad("robot_FR", 0.297,  0.314, 0.0))
        transforms.append(self.robot_broad("robot_RR", -0.297, 0.314, 0.0))
        transforms.append(self.robot_broad("robot_RL", -0.297,-0.314, 0.0))
        self.tf.sendTransform(transforms)   # static broad

    def callback_imu(self,msg):
        self.qx  = msg.orientation.x
        self.qy = msg.orientation.y
        self.qz = msg.orientation.z
        self.qw = msg.orientation.w
        self.broad_base()

     
    def broad_base(self):                # tilted ground -> base (broadcaster)
        # 조향 방향 갱신
        dtheta = 0.04
        self.theta_base += self.base_direct* dtheta

        # 범위 초과 시 방향 반전
        if self.theta_base <= self.theta_min or self.theta_base >= self.theta_max:
            self.base_direct *= -1

        # ground 회전 (Roll & Pitch)
        q = quaternion_from_euler(0, 0, self.theta_base)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = self.qx
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
        x = -0.1
        M = 25.0
        v = 0.0    # 최대속도 0.2m/s
        w = 0.0    # 최대각속도 0.8rad/s
        t = 0.5    # 도달시간 디폴드값

        omega = np.array([0.0, 0.0, w])
        alpha = np.array([0.0, 0.0, w / t])
        CoG_local = np.array([x, 0.0, h])
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
        g_local = R_world_to_local @ g_world 
         
        # non_force zmp
        x_nf = (M * g_local[2] * CoG_local[0] - M * g_local[0] * CoG_local[2]) / (M * g_local[2])
        y_nf = (M * g_local[2] * CoG_local[1] - M * g_local[1] * CoG_local[2]) / (M * g_local[2])
        self.get_logger().info(f"g좌표: {g_local}")
        # 구심가속도, 회전접선가속도
        a_c_local = np.cross(omega, np.cross(omega, CoG_local))
        circle_ac_local = np.cross(alpha,CoG_local)
        # 다음 수식은 결과 값임으로 abs(g_local)을 사용해야함.
        x_zmp_local = x_nf - h * (CoG_a_local[0] + a_c_local[0]) / abs(g_local[2])
        y_zmp_local = y_nf - h * (CoG_a_local[1] + a_c_local[1] + circle_ac_local[1]) / abs(g_local[2])
        zmp_local = np.array([x_zmp_local, y_zmp_local, 0.0])

        self.draw_marker("base","zmp", 200, zmp_local[0], zmp_local[1], 0.0, [0., 0., 0., 1.], Marker.SPHERE, [0.0, 0.0, 1.0], 0.07)


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