import sys
import pathlib
import time

import serial
import serial.tools.list_ports

# repo-root(상위 폴더)를 import 경로에 추가 -> common 패키지 사용
_REPO_ROOT = str(pathlib.Path(__file__).resolve().parent.parent)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from common.config import MOTOR_PORT, MOTOR_BAUDRATE, SERIAL_TIMEOUT
from motor_serial import send_command

# 포트/통신 설정은 common/config.py 에서 관리한다.
# 실행 후 출력되는 포트 목록을 보고 config.py 의 MOTOR_PORT 를 수정하세요.

# 모터 2개 테스트 기준
# FORWARD: 왼쪽 + 오른쪽 모터 전진
# TURN_LEFT: 오른쪽 모터만 전진
# TURN_RIGHT: 왼쪽 모터만 전진
# STOP: 양쪽 모터 정지
KEY_COMMANDS = {
    "w": "FORWARD",
    "a": "TURN_LEFT",
    "d": "TURN_RIGHT",
    "s": "STOP",
}


def show_ports():
    print("사용 가능한 COM 포트")
    print("--------------------")

    for port in serial.tools.list_ports.comports():
        print(f"{port.device}: {port.description}")

    print("--------------------")
    print(f"현재 Arduino 포트 설정: {MOTOR_PORT}")
    print()


def main():
    show_ports()

    try:
        arduino = serial.Serial(MOTOR_PORT, MOTOR_BAUDRATE, timeout=SERIAL_TIMEOUT)
        time.sleep(2)
        print("Arduino 연결됨")
    except Exception as exc:
        print("Arduino 연결 실패")
        print(exc)
        return

    print()
    print("키 입력 후 Enter")
    print("w: FORWARD     양쪽 모터 전진")
    print("a: TURN_LEFT   오른쪽 모터만 전진")
    print("d: TURN_RIGHT  왼쪽 모터만 전진")
    print("s: STOP        양쪽 모터 정지")
    print("e: 엔코더 카운트 조회 (ENC)")
    print("exit: 종료")
    print()

    last_command = None

    try:
        while True:
            key = input("명령> ").strip().lower()

            if key == "exit":
                send_command(arduino, "STOP")
                break

            if key == "e":
                arduino.reset_input_buffer()
                send_command(arduino, "ENC")
                line = arduino.readline().decode("utf-8", "ignore").strip()
                print(f"엔코더 카운트: {line}")
                continue

            if key not in KEY_COMMANDS:
                print("알 수 없는 명령입니다. w/a/d/s/e/exit 중 하나를 입력하세요.")
                continue

            command = KEY_COMMANDS[key]

            if command == last_command:
                print(f"같은 명령 유지: {command}")
                continue

            send_command(arduino, command)
            last_command = command

    except KeyboardInterrupt:
        print()
        send_command(arduino, "STOP")

    finally:
        arduino.close()
        print("Arduino 연결 종료")


if __name__ == "__main__":
    main()
