import rclpy as rp
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import TransformStamped, Point
import tf2_ros
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_matrix
from rclpy.duration import Duration as rclpyDuration  # timeout용
# test
class BaseBroad(Node):
    
    def __init__(self):
        super().__init__("Basebroad_node")
        # IMU구독용
        self.imu_sub = self.create_subscription(Imu, "/imu/data",self.callback_imu,10)
        # 마커 pub용
        self.marker = self.create_publisher(Marker, 'visualization_marker', 10)
        # broadcaster용
        self.tf = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_base = tf2_ros.TransformBroadcaster(self)
        self.tf_track = tf2_ros.TransformBroadcaster(self)

        #Marker용 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        transforms = []
        transforms.append(self.robot_broad("CoG", -0.1, 0.0, 0.135))
        transforms.append(self.robot_broad("robot_FL", 0.297, -0.314, 0.0))
        transforms.append(self.robot_broad("robot_FR", 0.297,  0.314, 0.0))
        transforms.append(self.robot_broad("robot_RR", -0.297, 0.314, 0.0))
        transforms.append(self.robot_broad("robot_RL", -0.297,-0.314, 0.0))
        self.tf.sendTransform(transforms)

    def callback_imu(self,msg):
        self.qx  = msg.orientation.x
        self.qy = msg.orientation.y
        self.qz = msg.orientation.z
        self.qw = msg.orientation.w
        self.broad_base()

     
    def broad_base(self):          

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

        self.broad_track()
        self.draw_marker("robtf", 1, -0.1, 0.0, 0.135, [0., 0., 0., 1.], Marker.SPHERE, [0.0, 1.0, 0.0], 0.1)
        self.draw_rectangle_marker()
        self.draw_zmp_marker()
        self.tf_base.sendTransform(t)
    def broad_track(self):          

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base"
        t.child_frame_id = "track_Right"
        t.transform.translation.x = 0.297
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.135
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_track.sendTransform(t)

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

    def draw_marker(self, ns, marker_id, x, y, z, quat, marker_type, rgb, scale, lifetime=0):
        m = Marker()
        m.header.frame_id = "base"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
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
        names = ["robot_FL", "robot_FR", "robot_RR", "robot_RL", "robot_FL"]
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
            #R = euler_from_quaternion(q)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed for world → base: {e}")
            return

        # psi = R[0]  # Roll
        # th = R[1]   # pitch
        # yaw = R[2]  # yaw

        # R_pitch = np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])
        # R_roll = np.array([[1, 0, 0], [0, math.cos(psi), -math.sin(psi)], [0, math.sin(psi), math.cos(psi)]])
        # R_yaw = np.array([[math.cos(yaw),-math.sin(yaw), 0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
        # R_world_to_local = R_yaw @ R_pitch @ R_roll  # ros회전변환순서 ZYX
        R_world_to_local = quaternion_matrix(q)[:3,:3]
        g_world = np.array([0.0, 0.0, g])
        g_local = R_world_to_local @ g_world 
 
        x_nf = (M * g_local[2] * CoG_local[0] - M * g_local[0] * CoG_local[2]) / (M * g_local[2])
        y_nf = (M * g_local[2] * CoG_local[1] - M * g_local[1] * CoG_local[2]) / (M * g_local[2])

        a_c_local = np.cross(omega, np.cross(omega, CoG_local))
        circle_ac_local = np.cross(alpha,CoG_local)
        # 다음 수식은 결과 값임
        x_zmp_local = x_nf - h * (CoG_a_local[0] + a_c_local[0]) / (g_local[2])
        y_zmp_local = y_nf - h * (CoG_a_local[1] + a_c_local[1] + circle_ac_local[1]) / (g_local[2])
        zmp_local = np.array([x_zmp_local, y_zmp_local, 0.0])

        self.draw_marker("zmp", 200, zmp_local[0], zmp_local[1], 0.0, [0., 0., 0., 1.], Marker.SPHERE, [0.0, 0.0, 1.0], 0.07)




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