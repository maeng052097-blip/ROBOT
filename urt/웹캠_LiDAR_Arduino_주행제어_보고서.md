# 웹캠 LiDAR Arduino 주행제어 보고서

## 1. 목적

본 시스템의 목적은 웹캠과 LiDAR를 이용하여 목표 물체를 추적하면서, Arduino 기반 모터 제어를 통해 차체를 주행시키는 것이다.

웹캠은 목표 물체의 방향을 판단하고, LiDAR는 차체 주변의 안전 상태를 판단한다. 최종 주행 명령은 두 센서의 판단 결과를 통합하여 생성하며, Arduino는 해당 명령을 받아 실제 모터를 제어한다.

## 2. 전체 시스템 구성

```text
웹캠
-> 목표 물체 인식
-> 목표 방향 판단

LiDAR
-> 차체 주변 장애물 감지
-> 안전박스 판단

Python / VS Code
-> 웹캠 판단 + LiDAR 판단 통합
-> 최종 주행 명령 생성
-> Arduino로 시리얼 명령 전송

Arduino
-> 명령 문자열 수신
-> 모터 드라이버 제어
-> 좌우 DC 모터 동작
```

## 3. 센서 역할

웹캠 역할:

```text
- 차체 정면에 장착
- 목표 물체를 인식
- 목표 물체가 화면 왼쪽, 중앙, 오른쪽 중 어디에 있는지 판단
- 주행 방향 결정에 사용
```

LiDAR 역할:

```text
- 차체 정중앙에 장착
- 차체 주변 360도 거리 감지
- 안전박스 안에 장애물이 들어왔는지 판단
- 감속 또는 정지 조건 생성
```

## 4. 판단 우선순위

주행 판단에서는 LiDAR 안전 판단이 웹캠 목표 추적보다 우선한다.

```text
1순위: LiDAR 안전 판단
- 위험 범위 안에 장애물 있음 -> STOP
- 감속 범위 안에 장애물 있음 -> SLOW 또는 제한 주행

2순위: 웹캠 목표 추적
- 목표가 왼쪽 -> TURN_LEFT
- 목표가 중앙 -> FORWARD
- 목표가 오른쪽 -> TURN_RIGHT

3순위: 목표 없음
- SEARCH 또는 STOP
```

## 5. 기본 주행 명령

현재 2모터 테스트 기준에서는 아래 4개 명령을 우선 사용한다.

```text
FORWARD
TURN_LEFT
TURN_RIGHT
STOP
```

확장 시 사용할 수 있는 명령:

```text
SLOW_FORWARD
AVOID_LEFT
AVOID_RIGHT
SEARCH
```

## 6. 2모터 동작 기준

현재 테스트 구조는 좌우 DC 모터 2개를 사용한다.

```text
FORWARD
-> 왼쪽 모터 전진
-> 오른쪽 모터 전진

TURN_LEFT
-> 왼쪽 모터 정지
-> 오른쪽 모터 전진

TURN_RIGHT
-> 왼쪽 모터 전진
-> 오른쪽 모터 정지

STOP
-> 왼쪽 모터 정지
-> 오른쪽 모터 정지
```

즉, 좌회전은 오른쪽 모터만 돌려 차체가 왼쪽으로 틀어지게 하고, 우회전은 왼쪽 모터만 돌려 차체가 오른쪽으로 틀어지게 한다.

## 7. Python에서 Arduino로 보낼 명령 형식

Python은 Arduino에 문자열 명령을 시리얼로 전송한다.

예시:

```text
FORWARD\n
TURN_LEFT\n
TURN_RIGHT\n
STOP\n
```

Arduino는 줄바꿈 문자 `\n`을 기준으로 명령을 읽는다.

Python 측 전송 예시:

```python
arduino.write((command + "\n").encode("utf-8"))
```

## 8. Arduino 코드에서 구현해야 할 처리

Arduino는 시리얼 입력을 읽고 명령 문자열에 따라 모터 드라이버를 제어해야 한다.

필요한 명령 처리:

```text
if command == "FORWARD":
    양쪽 모터 전진

elif command == "TURN_LEFT":
    오른쪽 모터만 전진

elif command == "TURN_RIGHT":
    왼쪽 모터만 전진

elif command == "STOP":
    양쪽 모터 정지
```

## 9. VS Code Python 코드와 Arduino 코드의 연결 방식

연결 구조:

```text
Python 실행 PC
-> USB Serial
-> Arduino
-> Motor Driver
-> Left Motor / Right Motor
```

주의사항:

```text
LiDAR와 Arduino는 서로 다른 COM 포트를 사용해야 한다.

예:
LiDAR   -> COM3
Arduino -> COM4
```

Python 코드에서는 Arduino 포트를 별도로 설정해야 한다.

```python
MOTOR_PORT = "COM4"
MOTOR_BAUDRATE = 115200
```

Arduino 코드에서도 동일한 baudrate를 사용해야 한다.

```cpp
Serial.begin(115200);
```

## 10. 현재 테스트용 Python 파일

현재 생성된 모터 테스트용 Python 파일:

```text
모터2개_시리얼테스트.py
```

역할:

```text
- 센서 없이 Arduino 모터 통신만 테스트
- 키보드 입력으로 명령 전송
```

키 입력:

```text
w -> FORWARD
a -> TURN_LEFT
d -> TURN_RIGHT
s -> STOP
exit -> 종료
```

이 파일은 Arduino 모터 코드가 정상 동작하는지 먼저 확인하기 위한 용도이다.

## 11. 최종 통합 시 필요한 구조

최종 통합 Python 코드는 다음 흐름을 가져야 한다.

```text
1. 웹캠 프레임 읽기
2. 목표 물체 인식
3. 목표 방향 판단
4. LiDAR 거리 데이터 읽기
5. 안전박스 판단
6. 최종 주행 명령 생성
7. 이전 명령과 다를 경우 Arduino로 전송
```

권장 최종 흐름:

```python
camera_state = detect_target_from_webcam()
lidar_state = analyze_lidar_safety()
command = decide_drive_command(camera_state, lidar_state)

if command != last_command:
    send_command_to_arduino(command)
    last_command = command
```

## 12. 통합 판단 예시

```text
LiDAR 위험 감지
-> STOP

LiDAR 안전 + 웹캠 목표 왼쪽
-> TURN_LEFT

LiDAR 안전 + 웹캠 목표 중앙
-> FORWARD

LiDAR 안전 + 웹캠 목표 오른쪽
-> TURN_RIGHT

웹캠 목표 없음
-> STOP 또는 SEARCH
```

## 13. 다른 AI 모듈에 전달할 핵심 요구사항

다른 AI 모듈이 Arduino 코드를 작성할 때 반영해야 할 핵심 조건은 다음과 같다.

```text
- Python에서 문자열 명령을 Serial로 전송한다.
- Arduino는 Serial.readStringUntil('\n') 방식으로 명령을 읽는다.
- 명령은 FORWARD, TURN_LEFT, TURN_RIGHT, STOP 네 가지를 우선 지원한다.
- 모터는 좌우 2개이다.
- FORWARD는 양쪽 모터 전진이다.
- TURN_LEFT는 오른쪽 모터만 전진이다.
- TURN_RIGHT는 왼쪽 모터만 전진이다.
- STOP은 양쪽 모터 정지이다.
- baudrate는 115200으로 맞춘다.
```

## 14. 요약

본 시스템은 웹캠이 목표 물체의 방향을 판단하고, LiDAR가 차체 주변의 위험을 판단한 뒤, Python 코드가 최종 주행 명령을 생성하여 Arduino에 전달하는 구조이다.

Arduino는 Python에서 받은 명령 문자열에 따라 좌우 두 개의 모터를 제어한다. 초기 테스트에서는 `FORWARD`, `TURN_LEFT`, `TURN_RIGHT`, `STOP` 네 가지 명령만 사용하며, 이후 필요에 따라 `SLOW_FORWARD`, `AVOID_LEFT`, `AVOID_RIGHT`, `SEARCH` 명령을 확장할 수 있다.
