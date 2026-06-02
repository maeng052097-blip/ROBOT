"""
웹캠 + LiDAR 통합 주행제어

흐름 (보고서 11장):
  1. 웹캠 프레임 읽기      -> 목표 방향 판단 (LEFT / CENTER / RIGHT / NONE)
  2. LiDAR 거리 읽기       -> 안전박스 판단 (DANGER / SLOW / SAFE)
  3. 두 판단을 우선순위로 통합 -> 최종 주행 명령
  4. 명령이 바뀔 때만 Arduino로 전송

현재 상태:
  - analyze_lidar_safety  : drivers.LidarX2 로 전방 부채꼴 최소거리를 읽어 구현됨 (Phase 1)
  - detect_target_from_webcam : 아직 자리표시자(항상 NONE). YOLO 가중치 확보 후 채운다 (Phase 2)
  - 설정값(포트/임계값 등)은 common/config.py 에서 관리한다.
"""

import sys
import pathlib
import time

# repo-root(상위 폴더)를 import 경로에 추가 -> common / drivers 패키지 사용
_REPO_ROOT = str(pathlib.Path(__file__).resolve().parent.parent)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from common.config import (
    MOTOR_PORT, MOTOR_BAUDRATE, LIDAR_PORT, LIDAR_BAUDRATE, SERIAL_TIMEOUT,
    CAMERA_INDEX, LOOP_DELAY, WEIGHTS_PATH, REQUIRE_LIDAR,
)
from common.safety import classify_safety
from drivers.LidarX2 import LidarX2
from inference.detector import TargetDetector
from motor_serial import send_command

# 웹캠 목표 방향 / LiDAR 안전 상태 값
CAMERA_LEFT, CAMERA_CENTER, CAMERA_RIGHT, CAMERA_NONE = "LEFT", "CENTER", "RIGHT", "NONE"
LIDAR_DANGER, LIDAR_SLOW, LIDAR_SAFE = "DANGER", "SLOW", "SAFE"


def detect_target_from_webcam(cap, detector):
    """웹캠 프레임을 읽어 목표 방향(LEFT/CENTER/RIGHT/NONE)을 판단한다.

    detector(TargetDetector)가 화면을 좌/중/우로 3등분해 가장 신뢰도 높은
    목표가 어느 구역에 있는지 돌려준다.
    """
    ret, frame = cap.read()
    if not ret:
        return CAMERA_NONE
    return detector.detect_direction(frame)


def analyze_lidar_safety(lidar):
    """LiDAR 전방 부채꼴 최소거리로 안전 상태(DANGER/SLOW/SAFE)를 판단한다.

    실제 판단 로직은 common.safety.classify_safety 에 있다
    (LiDAR 스모크 테스트 tests/test_scan.py 와 동일 로직 공유).
    """
    return classify_safety(lidar.getDistanceDict())


def decide_drive_command(camera_state, lidar_state):
    """LiDAR(안전) 우선, 웹캠(추적) 차순으로 최종 명령을 만든다 (보고서 4장).

    우선순위:
      1) LiDAR DANGER     -> 무조건 ESTOP (즉시정지, ramp 무시)
      2) 웹캠 목표 추적 (LEFT / RIGHT / CENTER)
         단, LiDAR SLOW 구간에서는 전방 직진(CENTER)만 막아 STOP 하고,
         좌/우 회전 추적은 허용한다 (정책 A).
         (펌웨어에 SLOW_FORWARD 가 추가되면 CENTER -> 감속 직진으로 승격 가능.)
      3) 목표 없음(NONE)  -> STOP
    """
    # 1순위: LiDAR 위험 -> 즉시 정지 (ESTOP: ramp 무시)
    if lidar_state == LIDAR_DANGER:
        return "ESTOP"

    # 2순위: 웹캠 목표 추적
    if camera_state == CAMERA_LEFT:
        return "TURN_LEFT"
    if camera_state == CAMERA_RIGHT:
        return "TURN_RIGHT"
    if camera_state == CAMERA_CENTER:
        # SLOW 구간에서는 전방 장애물이 가까우므로 직진하지 않는다.
        return "STOP" if lidar_state == LIDAR_SLOW else "FORWARD"

    # 3순위: 목표 없음
    return "STOP"


def _wait_for_ready(arduino, timeout=3.0):
    """Arduino 가 부팅 후 보내는 'READY' 신호를 기다린다(없어도 계속 진행).

    펌웨어(arduino_motor/src/main.cpp)는 setup() 에서 'READY' 를,
    명령마다 'OK: <cmd>' 를 보낸다.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = arduino.readline().decode("utf-8", "ignore").strip()
        if line:
            print(f"  Arduino: {line}")
            if line == "READY":
                return True
    return False


def main():
    # cv2 / serial 은 실제 장치 구동 시에만 필요하므로 여기서 임포트한다.
    # (이렇게 하면 판단 로직 함수만 따로 임포트/테스트할 수 있다.)
    import cv2
    import serial
    import serial.tools.list_ports

    # 연결 가능한 포트 안내
    print("사용 가능한 COM 포트:")
    for port in serial.tools.list_ports.comports():
        print(f"  {port.device}: {port.description}")
    print(f"  설정 -> MOTOR={MOTOR_PORT}, LIDAR={LIDAR_PORT}\n")

    arduino = None
    cap = None
    lidar = None
    detector = None
    try:
        # 1) Arduino (필수): 실패하면 모터 제어가 불가하므로 중단
        try:
            arduino = serial.Serial(MOTOR_PORT, MOTOR_BAUDRATE, timeout=SERIAL_TIMEOUT)
            time.sleep(2)  # 보드 리셋 대기
            _wait_for_ready(arduino)
            print(f"[OK] Arduino 연결: {MOTOR_PORT}")
        except Exception as exc:
            print(f"[중단] Arduino({MOTOR_PORT}) 연결 실패: {exc}")
            return

        # 2) LiDAR: 안전 판단의 핵심. REQUIRE_LIDAR 면 실패 시 중단,
        #    아니면 저하 모드(전방 직진 금지)로 진행.
        lidar = LidarX2(LIDAR_PORT, LIDAR_BAUDRATE)
        if lidar.open():
            print(f"[OK] LiDAR 연결: {LIDAR_PORT}")
        else:
            lidar.close()
            lidar = None
            if REQUIRE_LIDAR:
                print(f"[중단] LiDAR({LIDAR_PORT}) 연결 실패. 안전 판단이 불가하여 주행을 멈춥니다.")
                return
            print("[경고] LiDAR 없음 -> 저하 모드: 전방 직진 금지(SLOW 취급), 회전 추적만 허용")

        # 3) 카메라 (선택): 실패해도 목표 NONE -> STOP 으로 안전하게 동작
        cap = cv2.VideoCapture(CAMERA_INDEX)
        if cap.isOpened():
            print(f"[OK] 카메라 연결: index {CAMERA_INDEX}")
        else:
            print(f"[경고] 카메라(index {CAMERA_INDEX}) 열기 실패 -> 목표 인식 없이 진행")

        # 4) YOLO 목표 탐지기 (선택): 가중치가 없으면 목표 인식만 끄고 계속 진행
        try:
            detector = TargetDetector()
            print(f"[OK] YOLO 모델 로드: {WEIGHTS_PATH}")
        except FileNotFoundError as exc:
            print(f"[경고] YOLO 가중치 없음 -> 목표 인식 비활성화\n  {exc}")
        except Exception as exc:
            print(f"[경고] YOLO 로드 실패 -> 목표 인식 비활성화: {exc}")

        # LiDAR 워밍업: 첫 스캔이 들어올 때까지 잠깐 대기
        # (시작 직후 빈 데이터를 SAFE 로 오판하는 것을 막는다)
        if lidar is not None:
            warm_deadline = time.time() + 3.0
            while time.time() < warm_deadline and not lidar.getDistanceDict():
                time.sleep(0.1)

        print("\n주행 시작. Ctrl+C 로 종료.\n")

        last_command = None
        while True:
            camera_state = (
                detect_target_from_webcam(cap, detector)
                if (cap is not None and detector is not None) else CAMERA_NONE
            )
            # LiDAR 없으면(저하 모드) SLOW 로 취급 -> 직진 금지, 회전 추적만 허용
            lidar_state = analyze_lidar_safety(lidar) if lidar is not None else LIDAR_SLOW
            command = decide_drive_command(camera_state, lidar_state)

            if command != last_command:
                send_command(arduino, command)
                last_command = command

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print()

    finally:
        # 안전 최우선: 종료 시 반드시 모터 정지 후 장치 해제 (None 안전 처리)
        if arduino is not None:
            try:
                send_command(arduino, "ESTOP")  # 종료 시 즉시 정지
            except Exception:
                pass
        if cap is not None:
            cap.release()
        if lidar is not None:
            lidar.close()
        if arduino is not None:
            arduino.close()
        print("종료: 모터 정지 및 장치 해제")


if __name__ == "__main__":
    main()
