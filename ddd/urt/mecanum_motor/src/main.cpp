/*
  mecanum_motor / main.cpp  — 메카넘 4모터 '명령형' 펌웨어
  보드: Arduino Mega 2560 + Cytron MDD10A x2 (Sign-Magnitude: 채널당 DIR + PWM)

  PC(파이썬)가 시리얼로 속도명령을 보내면 4개 메카넘 휠을 구동한다.
  four_motor_test.ino 의 핀맵/기어비를 그대로 쓰되, '명령 수신 + 메카넘 믹싱 +
  소프트 가감속 + 안전(STOP/ESTOP/데드맨 타임아웃)'을 추가했다.

  배선(= four_motor_test.ino 와 동일):
    PWM : M1=5, M2=6, M3=7, M4=8
    DIR : M1=22, M2=23, M3=24, M4=25
    ENC_A(인터럽트): M1=2, M2=3, M3=18, M4=19   (A채널만 -> 부호는 명령방향으로)
    ※ 각 A 라인 외부 1kΩ 풀업(5V). 두 MDD10A 의 B+/B- 배터리 병렬, GND 공통.

  휠 역할(기본 가정 — 장착 후 확인/수정): M1=앞왼(FL) M2=앞오(FR) M3=뒤왼(RL) M4=뒤오(RR)

  명령 규약 (개행 '\n' 종료, 115200):
    V <vx> <vy> <w>   속도명령. 각 -100~100(%) 정수.
                       vx=+전진, vy=+우strafe, w=+우회전(CW).
                       믹싱(common/mecanum.py 와 동일):
                         FL=vx+vy+w  FR=vx-vy-w  RL=vx-vy+w  RR=vx+vy-w  (초과시 비례축소)
    M <i> <pct>       단일 모터 테스트(i=1..4=M1..M4, pct=-100~100). 코너 매핑/방향 확인용.
    STOP              완만 감속 정지(ramp)
    ESTOP             즉시 정지(ramp 무시, 안전용)
    ENC               "ENC m1 m2 m3 m4" 카운트 응답
    ENCRESET          카운트 0 리셋 후 응답
  안전:
    - 데드맨: CMD_TIMEOUT_MS 동안 어떤 명령도 없으면 목표를 0 으로(자동 감속). PC 끊김 보호.
    - 부팅 시 정지, 부팅 후 "READY" 송신.
*/
#include <Arduino.h>
#include <util/atomic.h>
#include <stdlib.h>

const uint8_t N = 4;
const uint8_t PWM_PIN[N] = {5, 6, 7, 8};
const uint8_t DIR_PIN[N] = {22, 23, 24, 25};
const uint8_t ENC_A[N]   = {2, 3, 18, 19};   // 인터럽트 핀

// ===== 배선 보정 =====
// 전진(+) 명령에 해당 모터가 반대로 돌면 그 칸을 true 로(장착 후 확인).
// 측정: M1(앞오)=후진, M4(뒤오)=후진 -> 그 둘만 반전. M2,M3=전진(정상).
const bool    INVERT[N]  = {true, false, false, true};
// DIR 핀의 '전진' 레벨(가정). 전체가 반대로 가면 이 한 줄을 LOW 로.
const uint8_t FWD_LEVEL  = HIGH;
// 논리 코너(0=FL 앞왼, 1=FR 앞오, 2=RL 뒤왼, 3=RR 뒤오) -> 물리 모터 인덱스(0=M1..3=M4).
// 단일모터 테스트(M 1..4)로 각 모터가 어느 코너인지 확인한 뒤 이 표를 실제에 맞게 채운다.
// 측정 결과: M1=앞오(FR), M2=앞왼(FL), M3=뒤왼(RL), M4=뒤오(RR)
// 코너(FL,FR,RL,RR) -> 물리모터: FL=M2(1), FR=M1(0), RL=M3(2), RR=M4(3)
const uint8_t POS_TO_MOTOR[N] = {1, 0, 2, 3};
// strafe(vy) 방향 보정: 전진·회전은 맞는데 좌우 평행이동만 반대면 -1 (롤러/장착 핸들 차이).
// 측정: V 0 60 0(우 strafe)에 로봇이 좌로 감 -> -1.
const int8_t  STRAFE_SIGN = -1;

// ===== 구동 파라미터 =====
const uint8_t  MAX_DUTY       = 200;  // 100% 명령 -> 이 듀티(0~255). 속도/전류 상한.
// PWM 주파수 = 5kHz (요구사항). Fast PWM, TOP=ICR=PWM_TOP, prescaler=1, F_CPU=16MHz.
//   freq = 16,000,000 / (1*(1+3199)) = 5000 Hz. 듀티(0~255)는 0~PWM_TOP 로 환산해 OCR 에 씀.
const uint16_t PWM_TOP        = 3199;
const int      RAMP_STEP      = 8;    // 틱당 듀티 증감(소프트 가감속, 급반전 방지)
const uint16_t TICK_MS        = 15;
const uint16_t CMD_TIMEOUT_MS = 1500; // 데드맨: 이 시간 명령 없으면 정지(벤치 관찰용 1.5s.
                                      //   자율주행은 PC가 ~30ms마다 명령을 보내 무관. 빠른
                                      //   주행에서 더 안전하게 하려면 400~600 으로 낮춰도 됨)

// ===== 엔코더 =====
volatile unsigned long count[N] = {0, 0, 0, 0};
volatile int8_t        encDir[N] = {1, 1, 1, 1};   // +1 전진 / -1 후진 (명령부호)
void isr0() { count[0] += encDir[0]; }
void isr1() { count[1] += encDir[1]; }
void isr2() { count[2] += encDir[2]; }
void isr3() { count[3] += encDir[3]; }
void (*const ISR_FN[N])() = {isr0, isr1, isr2, isr3};

int      target[N]  = {0, 0, 0, 0};   // 목표 듀티(-MAX_DUTY..MAX_DUTY)
int      current[N] = {0, 0, 0, 0};   // 현재 적용 듀티
uint32_t lastTick = 0, lastCmd = 0;

int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// (vx,vy,w in -100..100) -> 4휠 목표듀티. 논리코너(FL,FR,RL,RR) 계산 후 POS_TO_MOTOR 로 매핑.
// 합성이 100% 넘으면 비례 축소(방향 보존).
void mixSet(int vx, int vy, int w) {
  long vys = (long)vy * STRAFE_SIGN;   // strafe 축 부호 보정(롤러/장착 핸들)
  long logical[N] = {
    (long)vx + vys + w,   // FL
    (long)vx - vys - w,   // FR
    (long)vx - vys + w,   // RL
    (long)vx + vys - w    // RR
  };
  long m = max(max(labs(logical[0]), labs(logical[1])), max(labs(logical[2]), labs(logical[3])));
  float s = (m > 100) ? (100.0f / (float)m) : 1.0f;
  float k = (float)MAX_DUTY / 100.0f;
  for (uint8_t pos = 0; pos < N; pos++)
    target[POS_TO_MOTOR[pos]] = clampi((int)(logical[pos] * s * k), -MAX_DUTY, MAX_DUTY);
}

// PWM 5kHz 타이머 설정: Timer3(OC3A=pin5) + Timer4(OC4A/B/C=pin6/7/8).
//   Fast PWM, TOP=ICR=PWM_TOP, prescaler=1 -> 5kHz. 비반전(COMx1).
//   pins 2,3 은 엔코더 입력이라 OC3B/OC3C 출력은 켜지 않는다(외부 인터럽트와 무관).
void setupPwm5kHz() {
  TCCR3A = _BV(COM3A1) | _BV(WGM31);
  TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS30);
  ICR3 = PWM_TOP; OCR3A = 0;
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1) | _BV(WGM41);
  TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS40);
  ICR4 = PWM_TOP; OCR4A = OCR4B = OCR4C = 0;
}

// 듀티(0..255 스케일) -> 해당 모터 OCR(0..PWM_TOP) 직접 쓰기 (5kHz).
//   M1=pin5=OC3A, M2=pin6=OC4A, M3=pin7=OC4B, M4=pin8=OC4C
void writePwm(uint8_t i, uint16_t mag255) {
  uint16_t ocr = (uint16_t)((uint32_t)mag255 * PWM_TOP / 255);
  switch (i) {
    case 0: OCR3A = ocr; break;
    case 1: OCR4A = ocr; break;
    case 2: OCR4B = ocr; break;
    case 3: OCR4C = ocr; break;
  }
}

void applyWheel(uint8_t i, int duty) {
  bool fwd = (duty >= 0);
  int  mag = fwd ? duty : -duty;
  bool level = fwd ^ INVERT[i];                       // 전진여부 ^ 배선반전
  digitalWrite(DIR_PIN[i], level ? FWD_LEVEL : (FWD_LEVEL == HIGH ? LOW : HIGH));
  encDir[i] = fwd ? 1 : -1;
  writePwm(i, (uint16_t)mag);                         // 5kHz OCR 쓰기
}

int rampToward(int cur, int tgt, int step) {
  if (cur < tgt) { cur += step; if (cur > tgt) cur = tgt; }
  else if (cur > tgt) { cur -= step; if (cur < tgt) cur = tgt; }
  return cur;
}

void updateMotors() {
  for (uint8_t i = 0; i < N; i++) {
    current[i] = rampToward(current[i], target[i], RAMP_STEP);
    applyWheel(i, current[i]);
  }
}

void doEstop() {
  for (uint8_t i = 0; i < N; i++) { target[i] = 0; current[i] = 0; writePwm(i, 0); }
}

void printEnc() {
  long c[N];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { for (uint8_t i = 0; i < N; i++) c[i] = count[i]; }
  Serial.print("ENC");
  for (uint8_t i = 0; i < N; i++) { Serial.print(' '); Serial.print(c[i]); }
  Serial.println();
}

void handle(const String &cmd) {
  if (cmd.startsWith("V")) {
    int vx = 0, vy = 0, w = 0;
    if (sscanf(cmd.c_str() + 1, "%d %d %d", &vx, &vy, &w) == 3) {
      mixSet(clampi(vx, -100, 100), clampi(vy, -100, 100), clampi(w, -100, 100));
      lastCmd = millis();
      Serial.println("OK V");
    } else {
      Serial.println("ERR V");
    }
  } else if (cmd.startsWith("M")) {
    // 단일 모터 테스트: "M <i> <pct>"  (i=1..4=M1..M4, pct=-100..100). 나머지 모터는 0.
    int idx = 0, pct = 0;
    if (sscanf(cmd.c_str() + 1, "%d %d", &idx, &pct) == 2 && idx >= 1 && idx <= N) {
      for (uint8_t i = 0; i < N; i++) target[i] = 0;
      target[idx - 1] = clampi((int)(clampi(pct, -100, 100) * ((float)MAX_DUTY / 100.0f)),
                               -MAX_DUTY, MAX_DUTY);
      lastCmd = millis();
      Serial.print("OK M"); Serial.println(idx);
    } else {
      Serial.println("ERR M");
    }
  } else if (cmd == "STOP") {
    for (uint8_t i = 0; i < N; i++) target[i] = 0;   // ramp 로 감속
    lastCmd = millis();
    Serial.println("OK STOP");
  } else if (cmd == "ESTOP") {
    doEstop();
    lastCmd = millis();
    Serial.println("OK ESTOP");
  } else if (cmd == "ENC") {
    printEnc();
  } else if (cmd == "ENCRESET") {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { for (uint8_t i = 0; i < N; i++) count[i] = 0; }
    printEnc();
  } else {
    Serial.print("UNK "); Serial.println(cmd);
  }
}

void setup() {
  for (uint8_t i = 0; i < N; i++) {
    pinMode(PWM_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    digitalWrite(DIR_PIN[i], LOW);
    pinMode(ENC_A[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A[i]), ISR_FN[i], RISING);
  }
  setupPwm5kHz();                               // PWM 5kHz (요구사항), 듀티 0 으로 시작
  Serial.begin(115200);
  lastTick = lastCmd = millis();
  Serial.println("READY");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) handle(cmd);
  }

  uint32_t now = millis();
  // 데드맨: 명령이 끊기면 목표를 0 으로(완만 정지). PC 다운/케이블 분리 보호.
  if (now - lastCmd > CMD_TIMEOUT_MS) { for (uint8_t i = 0; i < N; i++) target[i] = 0; }

  if (now - lastTick >= TICK_MS) { lastTick = now; updateMotors(); }
}
