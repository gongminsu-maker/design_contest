#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS2 node: read "X: <roll>  Y: <pitch>  Z: <yaw_0_360>" from serial and publish /imu/data (sensor_msgs/Imu)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import serial
import time
import re
import math
from tf_transformations import quaternion_from_euler

SERIAL_PORT = '/dev/ttyACM0'    # 필요시 '/dev/ttyUSB0' 나 '/dev/serial0'로 변경
BAUD_RATE   = 115200            # 아두이노 Serial.begin 값과 동일!
TIMEOUT     = 1.0               # 읽기 타임아웃(초)

# "X: 12.3456    Y: -1.2345    Z: 300.0000" 만 잡도록 최소한의 정규식
XYZ_RE = re.compile(
    r'^\s*X:\s*([-+]?\d+(?:\.\d+)?)\s+Y:\s*([-+]?\d+(?:\.\d+)?)\s+Z:\s*([-+]?\d+(?:\.\d+)?)'
)

def deg2rad(x):
    return x * math.pi / 180.0

def wrap_yaw_deg_180(yaw_deg_0_360: float) -> float:
    """0~360° → -180~+180°로 래핑"""
    return ((yaw_deg_0_360 + 180.0) % 360.0) - 180.0

class ImuSerialPublisher(Node):
    def __init__(self):
        super().__init__('imu_serial_publisher')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        # 시리얼 열기
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
            time.sleep(2.0)  # 아두이노 리셋 대기
            self.get_logger().info(f"Opened {SERIAL_PORT} @ {BAUD_RATE}bps")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial open failed: {e}")
            raise

        # 타이머로 주기적 읽기
        self.timer = self.create_timer(0.0, self.read_once)  # 가능한 즉시 루프

    def read_once(self):
        raw = self.ser.readline()
        if not raw:
            return
        line = raw.decode('utf-8', errors='replace').strip()

        m = XYZ_RE.match(line)
        if not m:
            # 다른 라인이 섞이면 무시
            return

        # 아두이노가 보내는 의미 가정:
        # X -> roll(deg), Y -> pitch(deg), Z -> yaw(deg, 0~360)
        roll_deg  = float(m.group(1))
        pitch_deg = float(m.group(2))
        yaw_deg_0_360 = float(m.group(3))

        # yaw을 -180~+180으로 변환 후 라디안으로
        yaw_deg = wrap_yaw_deg_180(yaw_deg_0_360)

        roll  = deg2rad(roll_deg)
        pitch = deg2rad(pitch_deg)
        yaw   = deg2rad(yaw_deg)

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)  # (roll, pitch, yaw) 순서!

        msg = Imu()
        # stamp
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'imu_link'  # 필요에 맞게 변경

        # orientation
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # 가속/각속도는 지금 안 씀 → 0으로 두고 covariance는 미지(-1)로 표시
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(msg)

def main():
    rclpy.init()
    try:
        node = ImuSerialPublisher()
    except Exception:
        rclpy.shutdown()
        return

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
