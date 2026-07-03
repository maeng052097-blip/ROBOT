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
//    STOP        -> 양쪽 모터 정지 (부드럽게 감속)
//    ESTOP       -> 즉시 정지 (ramp 무시, 안전용)
//    ENC         -> 엔코더 카운트 응답 "ENC <left> <right>"
//    ENCRESET    -> 엔코더 카운트 0 으로 리셋 후 응답
//
//  엔코더: 단일 채널, 좌=핀2 / 우=핀3 (인터럽트). 방향은 현재 적용 속도 부호 기준.
//  주행: 명령은 '목표 속도'를 설정하고, loop 에서 부드럽게 가감속(ramp)한다.
//        -> 방향 전환 시 0 을 거쳐 감속 후 반대로 가속하므로 급격한 반전이 없다.
//  PWM 주파수: 7kHz (Timer4, Fast PWM, TOP=ICR4)
//  ※ 엔코더가 핀 2,3(인터럽트)을 사용하므로 모터 PWM 은 Timer4(핀 6,7)로 옮김.
// =============================================================

// ----- MDD10A 핀 연결 (Sign-Magnitude: 채널당 DIR + PWM) -----
// 좌측 모터 = 채널 1, 우측 모터 = 채널 2
// 엔코더가 핀 2,3(인터럽트 핀)을 쓰므로, 모터 PWM 은 Timer4 의 핀 6,7 을 사용한다.
// -> millis()용 Timer0과 충돌하지 않고, 한 타이머로 양쪽 주파수를 7kHz로 맞춘다.
//    ★ MDD10A 의 PWM 입력 2개를 아두이노 핀 6, 7 에 연결할 것 (기존 2,3 에서 이동).
const uint8_t LEFT_DIR  = 22;  // 디지털 출력
const uint8_t RIGHT_DIR = 23;  // 디지털 출력
const uint8_t LEFT_PWM  = 6;   // OC4A
const uint8_t RIGHT_PWM = 7;   // OC4B

// ----- 엔코더 (단일 채널: 모터당 신호선 1개) -----
// 앞바퀴 기준 좌 엔코더 -> 핀 2, 우 엔코더 -> 핀 3 (둘 다 Mega 인터럽트 핀).
// 단일 채널이라 회전 '방향'은 엔코더만으로 알 수 없어, 마지막 명령 방향으로 부호를 정한다.
const uint8_t LEFT_ENC  = 2;
const uint8_t RIGHT_ENC = 3;

volatile long encLeft  = 0;    // 좌 엔코더 누적 카운트(부호 포함)
volatile long encRight = 0;    // 우 엔코더 누적 카운트
volatile int8_t dirLeft  = 1;  // +1 전진 / -1 후진 (명령 기준)
volatile int8_t dirRight = 1;

// 엔코더 펄스 인터럽트 (상승 에지마다 1 카운트)
void onLeftPulse()  { encLeft  += dirLeft; }
void onRightPulse() { encRight += dirRight; }

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

// ----- 부드러운 가감속(ramp) -----
// 명령은 '목표 속도'만 정하고, 실제 속도는 RAMP_INTERVAL_MS 마다 RAMP_STEP 씩
// 목표로 다가간다. 방향 반전은 0 을 거쳐 일어나므로 급격한 반전이 없다.
// 더 빠릿한 반응을 원하면 RAMP_STEP 을 키우거나 RAMP_INTERVAL_MS 를 줄인다.
const int RAMP_STEP = 6;                    // 한 스텝당 속도 증감
const unsigned long RAMP_INTERVAL_MS = 15;  // 스텝 주기(ms). 약 (DRIVE_SPEED/STEP)*INTERVAL ≈ 300ms 로 0↔최고속.
int targetLeft = 0, targetRight = 0;        // 명령된 목표 속도 (-255..255)
int currentLeft = 0, currentRight = 0;      // 현재 적용 속도
unsigned long lastRampMs = 0;

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
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(WGM41);  // 비반전, Fast PWM 하위 비트
  TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS40);     // Fast PWM 상위 비트, 프리스케일러 1
  ICR4 = PWM_TOP;                                   // TOP -> 주파수 결정
  OCR4A = 0;                                        // 왼쪽 듀티 (핀 6)
  OCR4B = 0;                                        // 오른쪽 듀티 (핀 7)
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
// 명령은 '목표 속도'만 설정한다. 실제 출력은 updateMotors() 가 부드럽게 따라간다.
void drive(int leftSpeed, int rightSpeed) {
  targetLeft = leftSpeed;
  targetRight = rightSpeed;
}

// current 를 target 쪽으로 step 만큼 한 칸 이동(목표를 지나치지 않음)
int rampToward(int current, int target, int step) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}

// 주기적으로 호출: 현재 속도를 목표로 한 스텝 가감속하고 모터에 적용
void updateMotors() {
  currentLeft  = rampToward(currentLeft,  targetLeft,  RAMP_STEP);
  currentRight = rampToward(currentRight, targetRight, RAMP_STEP);

  // 엔코더 방향 부호 = 현재 적용 속도 부호 (0 이면 직전 값 유지)
  if (currentLeft  > 0) dirLeft  = 1; else if (currentLeft  < 0) dirLeft  = -1;
  if (currentRight > 0) dirRight = 1; else if (currentRight < 0) dirRight = -1;

  setMotor(LEFT_DIR, OCR4A, currentLeft);
  setMotor(RIGHT_DIR, OCR4B, currentRight);
}

// 엔코더 카운트를 "ENC <left> <right>" 형식으로 응답 (32비트 읽기는 인터럽트 보호)
void printEncoders() {
  noInterrupts();
  long l = encLeft, r = encRight;
  interrupts();
  Serial.print("ENC ");
  Serial.print(l);
  Serial.print(' ');
  Serial.println(r);
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
  } else if (cmd == "ESTOP") {
    // 즉시 정지(ramp 무시) — 안전용. 목표·현재 속도를 0 으로, 출력도 바로 0.
    targetLeft = 0;
    targetRight = 0;
    currentLeft = 0;
    currentRight = 0;
    setMotor(LEFT_DIR, OCR4A, 0);
    setMotor(RIGHT_DIR, OCR4B, 0);
  } else if (cmd == "ENC") {
    printEncoders();   // 현재 카운트만 응답하고 종료
    return;
  } else if (cmd == "ENCRESET") {
    noInterrupts();
    encLeft = 0;
    encRight = 0;
    interrupts();
    printEncoders();
    return;
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

  // 엔코더 입력 + 인터럽트 (단일 채널, 상승 에지마다 카운트)
  pinMode(LEFT_ENC, INPUT_PULLUP);
  pinMode(RIGHT_ENC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC), onLeftPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC), onRightPulse, RISING);

  drive(0, 0);  // 부팅 시 안전하게 정지 상태

  Serial.begin(115200);
  Serial.println("READY");
}

void loop() {
  // 줄바꿈(\n) 기준으로 한 줄씩 명령을 읽는다.
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();                 // 앞뒤 공백 / \r 제거
    if (cmd.length() > 0) applyCommand(cmd);
  }

  // 부드러운 가감속: 일정 주기마다 현재 속도를 목표로 한 스텝 이동
  unsigned long now = millis();
  if (now - lastRampMs >= RAMP_INTERVAL_MS) {
    lastRampMs = now;
    updateMotors();
  }
}
