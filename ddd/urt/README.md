# urt — 통합 주행 + 모터 제어

차체 주행 관련 코드를 담는다.

- `웹캠_LiDAR_주행제어.py` — 통합 주행 컨트롤러. 웹캠 목표 방향 + LiDAR 안전판단을
  합쳐 Arduino 로 주행 명령을 보낸다. 실행: `python urt/웹캠_LiDAR_주행제어.py`
- `모터2개_시리얼테스트.py` — 센서 없이 모터 2개만 수동 구동(키 w/a/d/s).
- `motor_serial.py` — Arduino 로 명령 문자열(`FORWARD\n` 등)을 보내는 공용 헬퍼.
- `arduino_motor/` — Arduino Mega 2560 펌웨어(PlatformIO). 명령:
  `FORWARD` / `BACKWARD` / `TURN_LEFT` / `TURN_RIGHT` / `STOP`.

설정(포트·임계값)은 `../common/config.py`, 전체 안내는 루트 `../README.md` 참고.
