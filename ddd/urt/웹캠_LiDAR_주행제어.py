"""
웹캠 + LiDAR 통합 주행제어 (골격)

흐름 (보고서 11장):
  1. 웹캠 프레임 읽기      -> 목표 방향 판단 (LEFT / CENTER / RIGHT / NONE)
  2. LiDAR 거리 읽기       -> 안전박스 판단 (DANGER / SLOW / SAFE)
  3. 두 판단을 우선순위로 통합 -> 최종 주행 명령
  4. 명령이 바뀔 때만 Arduino로 전송 (모터2개_시리얼테스트.py와 동일 패턴)

목표 인식(detect_target_from_webcam)과 거리 판단(analyze_lidar_safety)은
TODO 자리표시자다. 실제 모델/LiDAR SDK를 붙일 때 그 두 함수만 채우면 된다.
"""

import time

import cv2
import serial

from motor_serial import send_command

# ----- 포트 / 통신 설정 -----
# LiDAR와 Arduino는 서로 다른 COM 포트를 사용한다 (보고서 9장).
MOTOR_PORT = "COM4"
MOTOR_BAUDRATE = 115200
LIDAR_PORT = "COM3"
LIDAR_BAUDRATE = 115200
CAMERA_INDEX = 0
LOOP_DELAY = 0.05  # 초. 루프 주기.

# 웹캠 목표 방향 / LiDAR 안전 상태 값
CAMERA_LEFT, CAMERA_CENTER, CAMERA_RIGHT, CAMERA_NONE = "LEFT", "CENTER", "RIGHT", "NONE"
LIDAR_DANGER, LIDAR_SLOW, LIDAR_SAFE = "DANGER", "SLOW", "SAFE"


def detect_target_from_webcam(cap):
    """웹캠 프레임에서 목표 방향을 판단한다.

    화면을 좌/중/우 3등분해 목표 중심이 어느 구역에 있는지로 결정한다.
    지금은 인식 로직이 비어 있어 항상 NONE을 반환한다.
    """
    ret, frame = cap.read()
    if not ret:
        return CAMERA_NONE

    # TODO: 실제 목표 인식 (색상 마스크 / 딥러닝 모델 등)으로 목표 중심 x좌표를 구한다.
    #   width = frame.shape[1]
    #   if target_x < width / 3:        return CAMERA_LEFT
    #   elif target_x > width * 2 / 3:  return CAMERA_RIGHT
    #   else:                           return CAMERA_CENTER
    return CAMERA_NONE


def analyze_lidar_safety(lidar):
    """LiDAR 거리 데이터로 안전박스 상태를 판단한다.

    위험 범위에 장애물 -> DANGER, 감속 범위 -> SLOW, 그 외 -> SAFE.
    지금은 거리 파싱이 비어 있어 항상 SAFE를 반환한다.
    """
    # TODO: lidar에서 한 스캔의 거리값을 읽어 최소 거리를 구하고 임계값과 비교한다.
    #   if min_distance < DANGER_RANGE: return LIDAR_DANGER
    #   if min_distance < SLOW_RANGE:   return LIDAR_SLOW
    return LIDAR_SAFE


def decide_drive_command(camera_state, lidar_state):
    """LiDAR(안전) 우선, 웹캠(추적) 차순의 우선순위로 최종 명령을 만든다 (보고서 4장)."""
    # 1순위: LiDAR 안전 판단
    if lidar_state == LIDAR_DANGER:
        return "STOP"

    # 2순위: 웹캠 목표 추적
    if camera_state == CAMERA_LEFT:
        return "TURN_LEFT"
    if camera_state == CAMERA_RIGHT:
        return "TURN_RIGHT"
    if camera_state == CAMERA_CENTER:
        return "FORWARD"

    # 3순위: 목표 없음
    return "STOP"


def main():
    # 장치 연결
    arduino = serial.Serial(MOTOR_PORT, MOTOR_BAUDRATE, timeout=1.0)
    time.sleep(2)  # Arduino 리셋 대기
    cap = cv2.VideoCapture(CAMERA_INDEX)
    lidar = serial.Serial(LIDAR_PORT, LIDAR_BAUDRATE, timeout=1.0)
    print("장치 연결 완료. Ctrl+C 로 종료.")

    last_command = None
    try:
        while True:
            camera_state = detect_target_from_webcam(cap)
            lidar_state = analyze_lidar_safety(lidar)
            command = decide_drive_command(camera_state, lidar_state)

            if command != last_command:
                send_command(arduino, command)
                last_command = command

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print()

    finally:
        send_command(arduino, "STOP")
        cap.release()
        lidar.close()
        arduino.close()
        print("종료: 모터 정지 및 장치 해제")


if __name__ == "__main__":
    main()
