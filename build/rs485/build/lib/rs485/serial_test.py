import serial
import time

# 가짜 루프백 포트 열기 (pyserial 제공 기능)
ser = serial.serial_for_url('loop://', timeout=0.1)

# 데이터 쓰기 (보내면 그대로 버퍼에 다시 들어옴)
ser.write(b"HELLO")
print("[WRITE] HELLO")

time.sleep(0.1)  # 데이터가 버퍼로 들어갈 시간

# 현재 버퍼 상태 확인
print("[CHECK 1] in_waiting =", ser.in_waiting)

# 버퍼에 있는 만큼 다 읽기
n = ser.in_waiting
data = ser.read(n)
print(f"[READ {n}] ->", data)
print("[CHECK 2] in_waiting =", ser.in_waiting)

# 버퍼가 비었으니 다시 읽으면 어떻게 되나?
print("[READ 5] when buffer empty ->", ser.read(5))
print("[CHECK 3] in_waiting =", ser.in_waiting)

# 일부러 n = max(in_waiting, 1)로 읽어보기
ser.write(b"ABCD")
time.sleep(0.1)
n = max(ser.in_waiting, 1)
print(f"[CHECK 4] in_waiting = {ser.in_waiting}, n = {n}")
print("[READ n] ->", ser.read(n))
