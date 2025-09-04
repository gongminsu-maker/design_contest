import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import serial
import time
import re
import math
from tf_transformations import quaternion_from_euler

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
TIMEOUT = 1.0

# 정규식: X,Y,Z + R_r,P_r,Y_r + R_l,P_l,Y_l
LINE_RE = re.compile(
    r'X:\s*([-+]?\d+(?:\.\d+)?)\s+'
    r'Y:\s*([-+]?\d+(?:\.\d+)?)\s+'
    r'Z:\s*([-+]?\d+(?:\.\d+)?)'
    r'.*?R_r:\s*([-+]?\d+(?:\.\d+)?)\s+'
    r'P_r:\s*([-+]?\d+(?:\.\d+)?)\s+'
    r'Y_r:\s*([-+]?\d+(?:\.\d+)?)'
    r'.*?R_l:\s*([-+]?\d+(?:\.\d+)?)\s+'
    r'P_l:\s*([-+]?\d+(?:\.\d+)?)\s+'
    r'Y_l:\s*([-+]?\d+(?:\.\d+)?)'
)

def degrad(x: float) -> float:
    return x * math.pi / 180.0

def wrap_yaw_deg_180(yaw_deg_0_360: float) -> float:
    return ((yaw_deg_0_360 + 180.0) % 360.0) - 180.0

class ImuSerialPublisher(Node):
    def __init__(self):
        super().__init__('imu_serial_publisher')
        # Publisher 3개 생성
        self.pub_base  = self.create_publisher(Imu, '/base/imu/data', 10)
        self.pub_track_r = self.create_publisher(Imu, '/track_right/imu/data', 10)
        self.pub_track_l = self.create_publisher(Imu, '/track_left/imu/data', 10)

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
            time.sleep(2.0)  # 아두이노 리셋 대기
            self.get_logger().info(f"opened {SERIAL_PORT} @ {BAUD_RATE} bps")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial open failed : {e}")
            raise

        self.timer = self.create_timer(0.0, self.read_once)  # 가능한 즉시 루프

    def read_once(self):
        raw = self.ser.readline()
        if not raw:
            return
        line = raw.decode('utf-8', errors='replace').strip()

        m = LINE_RE.match(line)
        if not m:
            return

        # 값 파싱
        x = float(m.group(1))
        y = float(m.group(2))
        z = float(m.group(3))
        roll_r = float(m.group(4))
        pitch_r = float(m.group(5))
        yaw_r = float(m.group(6))
        roll_l = float(m.group(7))
        pitch_l = float(m.group(8))
        yaw_l = float(m.group(9))

        now = self.get_clock().now().to_msg()

        # ---- 1) BNO055 (base) ----
        z_180 = wrap_yaw_deg_180(z)
        x_rad, y_rad, z_rad = degrad(x), degrad(y), degrad(z_180)
        qx_b, qy_b, qz_b, qw_b = quaternion_from_euler(x_rad, y_rad, z_rad)

        msg_base = Imu()
        msg_base.header.stamp = now
        msg_base.header.frame_id = 'base'
        msg_base.orientation.x = qx_b
        msg_base.orientation.y = qy_b
        msg_base.orientation.z = qz_b
        msg_base.orientation.w = qw_b
        self.pub_base.publish(msg_base)

        # ---- 2) Track Right (MPU6050 0x69) ----
        roll_r_rad, pitch_r_rad, yaw_r_rad = degrad(roll_r), degrad(pitch_r), degrad(yaw_r)
        qx_r, qy_r, qz_r, qw_r = quaternion_from_euler(roll_r_rad, pitch_r_rad, yaw_r_rad)

        msg_r = Imu()
        msg_r.header.stamp = now
        msg_r.header.frame_id = 'track_right'
        msg_r.orientation.x = qx_r
        msg_r.orientation.y = qy_r
        msg_r.orientation.z = qz_r
        msg_r.orientation.w = qw_r
        self.pub_track_r.publish(msg_r)

        # ---- 3) Track Left (MPU6050 0x68) ----
        roll_l_rad, pitch_l_rad, yaw_l_rad = degrad(roll_l), degrad(pitch_l), degrad(yaw_l)
        qx_l, qy_l, qz_l, qw_l = quaternion_from_euler(roll_l_rad, pitch_l_rad, yaw_l_rad)

        msg_l = Imu()
        msg_l.header.stamp = now
        msg_l.header.frame_id = 'track_left'
        msg_l.orientation.x = qx_l
        msg_l.orientation.y = qy_l
        msg_l.orientation.z = qz_l
        msg_l.orientation.w = qw_l
        self.pub_track_l.publish(msg_l)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
