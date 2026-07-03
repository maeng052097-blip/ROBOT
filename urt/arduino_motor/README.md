# arduino_motor (PlatformIO)

웹캠/LiDAR 주행제어 시스템의 **Arduino 펌웨어** 프로젝트.
Python(PC)이 시리얼로 보낸 명령 문자열을 받아 좌/우 DC 모터 2개를 제어한다.

- 보드: **Arduino Mega 2560** (`board = megaatmega2560`)
- 모터 드라이버: **Cytron MDD10A** (Dual Channel, Sign-Magnitude 모드)
- baudrate: **115200** (Python `모터2개_시리얼테스트.py`와 동일해야 함)
- PWM 주파수: **7kHz** (Timer3 Fast PWM, TOP=ICR3, ≈6999Hz)
- 기본 주행 속도: **122 / 255** (≈48% 듀티)

## 명령 규약

| 명령 | 왼쪽 모터 | 오른쪽 모터 | 결과 |
|------|-----------|-------------|------|
| `FORWARD` | 전진 | 전진 | 직진 |
| `BACKWARD` | 후진 | 후진 | 후진 |
| `TURN_LEFT` | 정지 | 전진 | 좌회전 |
| `TURN_RIGHT` | 전진 | 정지 | 우회전 |
| `STOP` | 정지 | 정지 | 멈춤 |

PC는 `명령 + "\n"` 형식으로 전송하고, 펌웨어는 `Serial.readStringUntil('\n')`으로 읽는다.
처리 후 `OK: <명령>` / 알 수 없으면 `UNKNOWN: <명령>` 으로 응답한다.

## 배선 (MDD10A ↔ Mega 2560)

MDD10A는 **Sign-Magnitude 모드**(점퍼 기본값)에서 채널당 DIR + PWM 2핀을 쓴다.

| MDD10A 핀 | 연결 대상 | Mega 핀 |
|-----------|-----------|---------|
| CH1 DIR | 왼쪽 모터 방향 | 22 |
| CH1 PWM | 왼쪽 모터 속도 | 2 |
| CH2 DIR | 오른쪽 모터 방향 | 23 |
| CH2 PWM | 오른쪽 모터 속도 | 3 |
| GND | 공통 그라운드 | GND |

- 전원: MDD10A의 모터 전원 단자(B+/B-)에 **배터리**, 모터는 각 채널 출력 단자(M+/M-)에 연결.
- **반드시 Mega의 GND와 MDD10A의 GND를 공통 연결**해야 신호가 정상 인식된다.
- 모터가 반대로 돌면: `main.cpp`의 `FORWARD_LEVEL`을 `HIGH`로 바꾸거나 해당 모터 전선을 뒤집는다.
- PWM 핀(2, 3)은 Mega의 Timer3을 사용해 `millis()`용 Timer0과 충돌하지 않게 했다.

## 빌드 / 업로드

```bash
# 프로젝트 폴더에서
pio run                 # 컴파일
pio run -t upload       # Mega에 업로드
pio device monitor      # 시리얼 모니터 (115200)
```

> 주의: 업로드할 때와 Python 테스트를 돌릴 때 **같은 COM 포트를 동시에 점유할 수 없다.**
> 모니터/업로드 창을 닫은 뒤 Python을 실행한다.

## Python 테스트와 연동

1. 이 펌웨어를 Mega에 업로드한다.
2. 상위 폴더의 `모터2개_시리얼테스트.py`에서 `MOTOR_PORT`를 Mega의 COM 포트로 맞춘다.
3. 실행 후 `w/a/d/s` 키로 모터 동작을 확인한다.

## 설계 메모

- Python은 **명령이 바뀔 때만** 전송한다(보고서 11장). 따라서 펌웨어는 마지막 명령을
  계속 유지(latch)해야 하며, 이 코드는 그렇게 동작한다.
- 그래서 "일정 시간 명령이 없으면 정지" 같은 통신 타임아웃 페일세이프는 **넣지 않았다.**
  넣으면 FORWARD 유지 중에 차가 멈춰 설계와 충돌한다. 페일세이프가 필요하면 Python이
  주기적으로 하트비트를 보내는 구조로 바꾼 뒤 추가해야 한다.
