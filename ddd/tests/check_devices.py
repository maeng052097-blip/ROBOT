"""장치 연결 점검(preflight): Arduino / LiDAR / 웹캠이 모두 연결되는지 확인.

주행하지 않고 각 장치의 연결만 점검한다. 통합 주행 전에 실행을 권장한다.
"""
import sys
import pathlib
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from common.config import (
    MOTOR_PORT, MOTOR_BAUDRATE, LIDAR_PORT, LIDAR_BAUDRATE,
    SERIAL_TIMEOUT, CAMERA_INDEX, WEIGHTS_PATH,
)
from drivers.LidarX2 import LidarX2


def check_arduino():
    import serial
    ard = serial.Serial(MOTOR_PORT, MOTOR_BAUDRATE, timeout=SERIAL_TIMEOUT)
    try:
        time.sleep(2)  # 보드 리셋 대기
        line, deadline = "", time.time() + 3
        while time.time() < deadline:
            line = ard.readline().decode("utf-8", "ignore").strip()
            if line == "READY":
                break
        return True, f"{MOTOR_PORT} (READY {'수신' if line == 'READY' else '응답없음'})"
    finally:
        ard.close()


def check_lidar():
    lidar = LidarX2(LIDAR_PORT, LIDAR_BAUDRATE)
    if not lidar.open():
        return False, f"{LIDAR_PORT} open 실패"
    try:
        n, deadline = 0, time.time() + 3
        while time.time() < deadline:
            n = len(lidar.getDistanceDict())
            if n > 0:
                break
            time.sleep(0.1)
        return n > 0, f"{LIDAR_PORT} (스캔점 {n}개)"
    finally:
        lidar.close()


def check_camera():
    import cv2
    cap = cv2.VideoCapture(CAMERA_INDEX)
    try:
        if not cap.isOpened():
            return False, f"index {CAMERA_INDEX} 열기 실패"
        ret, frame = cap.read()
        if not ret:
            return False, f"index {CAMERA_INDEX} 프레임 수신 실패"
        return True, f"index {CAMERA_INDEX} (프레임 {frame.shape})"
    finally:
        cap.release()


def main():
    print("=== 장치 연결 점검 (preflight) ===")
    for name, fn in (("Arduino", check_arduino), ("LiDAR", check_lidar), ("웹캠", check_camera)):
        try:
            ok, info = fn()
        except Exception as exc:
            ok, info = False, str(exc)
        mark = "OK " if ok else "X  "
        print(f"  [{mark}] {name:8} {info}")
    print(f"  가중치 best.pt: {'있음' if WEIGHTS_PATH.exists() else '없음'}  ({WEIGHTS_PATH})")
    print("\n모터를 직접 돌려보려면:  python urt/모터2개_시리얼테스트.py  (w/a/d/s)")


if __name__ == "__main__":
    main()
