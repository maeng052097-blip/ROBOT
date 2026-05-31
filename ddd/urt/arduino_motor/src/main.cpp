#include <Arduino.h>

// =============================================================
//  웹캠 / LiDAR 주행제어 - 모터 펌웨어
//  보드        : Arduino Mega 2560
//  모터 드라이버 : Cytron MDD10A (Dual Channel, Sign-Magnitude 모드)
//
//  Python(PC) 이 "FORWARD\n" 같은 문자열을 시리얼로 보내면
//  이 펌웨어가 좌/우 DC 모터 2개를 제어한다.
//
//  명령 규약(보고서 6, 13장):
//    FORWARD     -> 양쪽 모터 전진
//    BACKWARD    -> 양쪽 모터 후진
//    TURN_LEFT   -> 오른쪽 모터만 전진 (왼쪽 정지) -> 차체 좌회전
//    TURN_RIGHT  -> 왼쪽 모터만 전진 (오른쪽 정지) -> 차체 우회전
//    STOP        -> 양쪽 모터 정지
//
//  PWM 주파수: 7kHz (Timer3, Fast PWM, TOP=ICR3)
// =============================================================

// ----- MDD10A 핀 연결 (Sign-Magnitude: 채널당 DIR + PWM) -----
// 좌측 모터 = 채널 1, 우측 모터 = 채널 2
// PWM 핀은 모두 Timer3(핀 2=OC3B, 핀 3=OC3C)을 사용한다.
// -> millis()용 Timer0과 충돌하지 않고, 한 타이머로 양쪽 주파수를 7kHz로 맞춘다.
const uint8_t LEFT_DIR  = 22;  // 디지털 출력
const uint8_t RIGHT_DIR = 23;  // 디지털 출력
const uint8_t LEFT_PWM  = 2;   // OC3B
const uint8_t RIGHT_PWM = 3;   // OC3C

// 전진을 만드는 DIR 레벨.
// 실제로 모터가 반대로 돌면 이 값을 HIGH로 바꾸거나, 모터 전선을 바꿔 끼운다.
const uint8_t FORWARD_LEVEL = LOW;
const uint8_t REVERSE_LEVEL = (FORWARD_LEVEL == LOW) ? HIGH : LOW;

// PWM 7kHz 설정값.
//   주파수 = F_CPU / (prescaler * (1 + TOP))
//          = 16,000,000 / (1 * (1 + 2285)) ≈ 6999 Hz
const uint16_t PWM_TOP = 2285;

// 기본 주행 속도 (0~255 스케일). 122/255 ≈ 48% 듀티.
const int DRIVE_SPEED = 122;

// 0~255 속도값을 현재 PWM_TOP 스케일의 OCR 값으로 변환
uint16_t speedToOcr(int speed) {
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;
  return (uint32_t)speed * PWM_TOP / 255;
}

// Timer3을 7kHz Fast PWM(TOP=ICR3)으로 설정. OC3B(핀2)/OC3C(핀3) 비반전 출력.
void setupPwm7kHz() {
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  TCCR3A = _BV(COM3B1) | _BV(COM3C1) | _BV(WGM31);  // 비반전, Fast PWM 하위 비트
  TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS30);     // Fast PWM 상위 비트, 프리스케일러 1
  ICR3 = PWM_TOP;                                   // TOP -> 주파수 결정
  OCR3B = 0;                                        // 왼쪽 듀티
  OCR3C = 0;                                        // 오른쪽 듀티
}

// 한 채널 제어. speed 범위 -255..255 (음수=후진, 0=정지)
void setMotor(uint8_t dirPin, volatile uint16_t &ocr, int speed) {
  if (speed >= 0) {
    digitalWrite(dirPin, FORWARD_LEVEL);
  } else {
    digitalWrite(dirPin, REVERSE_LEVEL);
    speed = -speed;
  }
  ocr = speedToOcr(speed);
}

// 좌/우 모터를 동시에 지정
void drive(int leftSpeed, int rightSpeed) {
  setMotor(LEFT_DIR, OCR3B, leftSpeed);
  setMotor(RIGHT_DIR, OCR3C, rightSpeed);
}

// 수신한 명령 문자열을 모터 동작으로 변환
void applyCommand(const String &cmd) {
  if (cmd == "FORWARD") {
    drive(DRIVE_SPEED, DRIVE_SPEED);
  } else if (cmd == "BACKWARD") {
    drive(-DRIVE_SPEED, -DRIVE_SPEED);
  } else if (cmd == "TURN_LEFT") {
    drive(0, DRIVE_SPEED);          // 왼쪽 정지, 오른쪽 전진
  } else if (cmd == "TURN_RIGHT") {
    drive(DRIVE_SPEED, 0);          // 왼쪽 전진, 오른쪽 정지
  } else if (cmd == "STOP") {
    drive(0, 0);
  } else {
    Serial.print("UNKNOWN: ");
    Serial.println(cmd);
    return;
  }

  // PC 쪽에서 동작을 확인할 수 있도록 응답
  Serial.print("OK: ");
  Serial.println(cmd);
}

void setup() {
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  setupPwm7kHz();

  drive(0, 0);  // 부팅 시 안전하게 정지 상태

  Serial.begin(115200);
  Serial.println("READY");
}

void loop() {
  // 줄바꿈(\n) 기준으로 한 줄씩 명령을 읽는다.
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();                 // 앞뒤 공백 / \r 제거
    if (cmd.length() == 0) return;
    applyCommand(cmd);
  }
}
