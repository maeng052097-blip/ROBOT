/*
  mecanum_motor / main.cpp  — 메카넘 4모터 '명령형' 펌웨어
  보드: Arduino Mega 2560 + Cytron MDD10A x2 (Sign-Magnitude: 채널당 DIR + PWM)

  PC(파이썬)가 시리얼로 속도명령을 보내면 4개 메카넘 휠을 구동한다.
  four_motor_test.ino 의 핀맵/기어비를 그대로 쓰되, '명령 수신 + 메카넘 믹싱 +
  소프트 가감속 + 안전(STOP/ESTOP/데드맨 타임아웃)'을 추가했다.

  배선 = MEGA_SHIELD_HKX (2026-09-29 이전. 설계문서 9-2. 옛 직결 배선 PWM 5~8 / DIR 22~25 / ENC 2,3,18,19 는 폐기):
    M1 = MD1 1채널: PWM D11 (MD1.2), DIR D34 (MD1.1), ENC1 A상 D3
    M2 = MD2 1채널: PWM D44 (MD2.2), DIR D36 (MD2.1), ENC2 A상 D18
    M3 = MD1 2채널: PWM D12 (MD1.4), DIR D35 (MD1.3), ENC3 A상 D19
    M4 = MD2 2채널: PWM D45 (MD2.4), DIR D37 (MD2.3), ENC4 A상 D20
    ENC B상(D30~D33)은 쉴드에 배선돼 있으나 쓰지 않는다(A상만 -> 부호는 명령방향으로).
    각 모터의 엔코더 선은 그 모터의 MD 채널과 같은 번호의 ENC 커넥터에 꽂는다(M1 모터 -> ENC1 ...).
    두 MDD10A 의 B+/B- 배터리 병렬, GND 공통.

  휠 역할(기본 가정 — 장착 후 확인/수정): M1=앞왼(FL) M2=앞오(FR) M3=뒤왼(RL) M4=뒤오(RR)

  명령 규약 (개행 '\n' 종료, 115200):
    V <vx> <vy> <w>   속도명령. 각 -100~100(%) 정수.
                       vx=+전진, vy=+우strafe, w=+우회전(CW).
                       믹싱(common/mecanum.py 와 동일):
                         FL=vx+vy+w  FR=vx-vy-w  RL=vx-vy+w  RR=vx+vy-w  (초과시 비례축소)
    M <i> <pct>       단일 모터 테스트(i=1..4=M1..M4, pct=-100~100). 코너 매핑/방향 확인용.
    P <i> <j> <pct>   두 모터 동시 테스트(2026-10-10). i, j = 1..4 서로 다름, pct=-100~100, 나머지 두 모터는 0.
                       INVERT 가 적용되므로 + 면 두 바퀴 모두 '전진' 방향. 응답 "OK P i j" / "ERR P".
    STOP              완만 감속 정지(ramp)
    ESTOP             즉시 정지(ramp 무시, 안전용)
    ENC               "ENC m1 m2 m3 m4" 카운트 응답
    ENCRESET          카운트 0 리셋 후 응답
    L                 램프 상태 조회 -> "LAMP s1 s2 s3 s4" (0=꺼짐 1=켜짐 2=점멸)
    L <n> <s>         램프 설정. n=1..4 (0=전체), s=0 꺼짐 / 1 켜짐 / 2 점멸(0.5s 간격).
                       응답은 조회와 같은 "LAMP ..." 한 줄. 인자가 틀리면 "ERR L".
  램프 배선: D14~D17 -> 쉴드 CN1.1~4 -> PCB-A J11.1~4 -> ULN2003 -> 릴레이 모듈 IN1~4
    HIGH = 릴레이 ON. 부팅 직후 전부 꺼짐. D14~D17 은 Serial3/Serial2 핀이라 두 포트는 쓰지 않는다.
    ID                "MECANUM" (보드 식별)
    PWR               동력 상태 조회 -> "PWR <ok> <latched> <cycled>"  (감시를 끈 펌웨어는 "PWR DISABLED")
                       ok 1 = 코일 ON 이 PWR_OK_STABLE_MS 이상 유지됨, latched 1 = 정지 잠김,
                       cycled 1 = 잠긴 뒤 코일 OFF 를 한 번 봤음
    CLR               동력 정지 잠김 해제. 조건: cycled 이고 ok. 응답:
                       "OK CLR" (해제, 잠겨 있지 않을 때도 이 응답) / "ERR PWR OFF" (지금 코일 OFF) /
                       "ERR PWR WAIT" (코일 ON 이지만 아직 안정 시간 전) /
                       "ERR PWR CYCLE" (잠긴 뒤 OFF 를 못 봄 -> 비상정지를 눌렀다 풀고 START 를 다시 누를 것)
    (비동기) "EVT PWR OFF"    코일 OFF 감지 (비상정지·전원 끊김·선 빠짐). OFF 상태로 바뀔 때마다 1회
    (비동기) "EVT PWR READY"  잠긴 상태에서 코일 ON 이 안정됨 = CLR 가능
  안전:
    - 데드맨: CMD_TIMEOUT_MS 동안 어떤 명령도 없으면 목표를 0 으로(자동 감속). PC 끊김 보호.
      램프(L) 명령은 데드맨을 갱신하지 않는다 -> 램프 명령만 오면 모터는 그대로 멈춘다.
    - 부팅 시 정지, 부팅 후 "READY" 송신.
    - 동력 감시 (2026-09-29): PCB-C J6 (옵토) -> A8. LOW = 접촉기 코일 ON, HIGH = 비상정지 / 전원 끊김 / 선 빠짐.
      HIGH 가 되는 순간 핀변화 인터럽트에서 PWM 출력을 타이머에서 떼어 LOW 로 고정한다(= MDD10A 브레이크).
      램프 감속을 하지 않는 이유: 접촉기가 열린 버스에서 듀티를 줄여 가면 모터 회생이 C1/C2 로 몰려
      MDD10A 한계(30V)를 넘을 수 있다. 코일 신호는 접촉기가 실제로 떨어지기 10~30ms 전에 바뀐다.
      잠김 중 V/M 명령은 "ERR PWR". 동력이 돌아와도 PC 가 CLR 을 보내기 전엔 움직이지 않는다.
      부팅하면 항상 잠긴 채 시작한다. 코일 OFF -> ON 을 실제로 한 번 봐야 CLR 이 된다.
        -> A8 이 LOW 에 붙어 있는 고장(GND 단락, 옵토 단락, 점퍼 잔존)이면 영영 풀리지 않아 드러난다.
        -> 접촉기가 이미 붙은 상태에서 아두이노만 리셋되면(PC 재접속 DTR 리셋 등) 비상정지 -> 해제 -> START 한 번 필요.
      ★PCB-C 없이(접촉기·비상정지 없는 임시 구성) 쓸 때는 PWR_SENSE_ENABLED 를 false 로 바꿔 업로드한다.
        이때 부팅 메시지에 "WARN PWR SENSE OFF" 가 나온다. PCB-C 를 달면 반드시 true 로 되돌린다.
*/
#include <Arduino.h>
#include <util/atomic.h>
#include <stdlib.h>

const uint8_t N = 4;
const uint8_t PWM_PIN[N] = {11, 44, 12, 45};  // Timer1 OC1A, Timer5 OC5C, OC1B, OC5B
const uint8_t DIR_PIN[N] = {34, 36, 35, 37};
const uint8_t ENC_A[N]   = {3, 18, 19, 20};   // 인터럽트 핀 (INT5, INT3, INT2, INT1)

// ===== 타워램프 (릴레이 4채널, PCB-A ULN2003 경유) =====
const uint8_t  NL = 4;
const uint8_t  LAMP_PIN[NL] = {14, 15, 16, 17};  // HIGH = 릴레이 ON
const uint16_t BLINK_HALF_MS = 500;              // 점멸 반주기
uint8_t        lampMode[NL] = {0, 0, 0, 0};      // 0=꺼짐 1=켜짐 2=점멸

// ===== 배선 보정 =====
// ★2026-10-10 바퀴 든 실측(받은 수동 펌웨어로 M1~M4 각각 +/-25%, Y2 쉴드, 오른쪽 드라이버 채널 교정 후):
//   M1 앞왼 / M3 뒤왼 은 + 에 전진, M2 앞오 / M4 뒤오 는 + 에 후진(좌우 모터가 마주 보게 장착) -> M2, M4 반전.
//   각 채널은 자기 엔코더 칸만 변함(ENC1~4 = M1~M4). STRAFE_SIGN 은 아직 바닥 미확인.
//   (옛 직결 배선 값 INVERT {1,0,0,1}, POS_TO_MOTOR {1,0,2,3}, STRAFE_SIGN -1 은 쓰지 않는다.)
// 전진(+) 명령에 해당 모터가 후진하면 그 칸을 true 로.
const bool    INVERT[N]  = {false, true, false, true};
// DIR 핀의 '전진' 레벨(가정). 전체가 반대로 가면 이 한 줄을 LOW 로.
const uint8_t FWD_LEVEL  = HIGH;
// 논리 코너(0=FL 앞왼, 1=FR 앞오, 2=RL 뒤왼, 3=RR 뒤오) -> 물리 모터 인덱스(0=M1..3=M4).
// 단일모터 테스트(M 1..4)로 각 모터가 어느 코너인지 확인한 뒤 이 표를 실제에 맞게 채운다.
// 2026-10-10 확인: M1=앞왼, M2=앞오, M3=뒤왼, M4=뒤오 (라이다 쪽이 뒤) -> {0, 1, 2, 3}.
constexpr uint8_t POS_TO_MOTOR[N] = {0, 1, 2, 3};
// strafe(vy) 방향 보정: 바퀴를 든 상태의 바퀴 패턴은 맞는데 '바닥에서' 좌우 평행이동만 반대면 -1.
//   바퀴를 든 상태에서 d(우 평행) 패턴이 반대면 이 값이 아니라 POS_TO_MOTOR 의 앞뒤(FL<->RL, FR<->RR)를 고친다.
const int8_t  STRAFE_SIGN = 1;
// 잘못 채우면(같은 모터 두 번 등) 컴파일이 멈춘다.
static_assert(POS_TO_MOTOR[0] < N && POS_TO_MOTOR[1] < N && POS_TO_MOTOR[2] < N && POS_TO_MOTOR[3] < N &&
              POS_TO_MOTOR[0] != POS_TO_MOTOR[1] && POS_TO_MOTOR[0] != POS_TO_MOTOR[2] &&
              POS_TO_MOTOR[0] != POS_TO_MOTOR[3] && POS_TO_MOTOR[1] != POS_TO_MOTOR[2] &&
              POS_TO_MOTOR[1] != POS_TO_MOTOR[3] && POS_TO_MOTOR[2] != POS_TO_MOTOR[3],
              "POS_TO_MOTOR must use each motor 0..3 exactly once");
static_assert(STRAFE_SIGN == 1 || STRAFE_SIGN == -1, "STRAFE_SIGN must be 1 or -1");

// ===== 구동 파라미터 =====
const uint8_t  MAX_DUTY       = 200;  // 100% 명령 -> 이 듀티(0~255). 속도/전류 상한.
// PWM 주파수 = 3.9kHz (요구사항). Fast PWM, TOP=ICR=PWM_TOP, prescaler=1, F_CPU=16MHz.
//   freq = 16,000,000 / (1*(1+4095)) = 3906.25 Hz. 듀티(0~255)는 0~PWM_TOP 로 환산해 OCR 에 씀.
const uint16_t PWM_TOP        = 4095;
const int      RAMP_STEP      = 8;    // 틱당 듀티 증감(소프트 가감속, 급반전 방지)
const uint16_t TICK_MS        = 15;
const uint16_t CMD_TIMEOUT_MS = 1500; // 데드맨: 이 시간 명령 없으면 정지(벤치 관찰용 1.5s.
                                      //   자율주행은 PC가 ~30ms마다 명령을 보내 무관. 빠른
                                      //   주행에서 더 안전하게 하려면 400~600 으로 낮춰도 됨)

// ===== 동력 감시 (PCB-C J6 -> A8, LOW = 코일 ON) =====
// A8 = PK0 = PCINT16 (PCINT2 벡터). 인터럽트에서 바로 출력을 끊어 loop 지연(시리얼 대기 등)과 무관하게 한다.
// 2026-09-29: 지금 하드웨어는 PCB-C 가 없는 임시 구성이라 false. ★PCB-C·접촉기·비상정지를 달면 true 로 바꿔 업로드.
const bool     PWR_SENSE_ENABLED = false; // false = PCB-C 없는 임시 구성 (동력 감시 끔, 부팅 경고)
const uint8_t  PWR_SENSE_PIN    = A8;
const uint16_t PWR_OK_STABLE_MS = 200;   // 코일 ON 이 이만큼 유지돼야 ok / CLR 허용 (접점 바운스 무시)
volatile bool  pwrTrip          = false; // 인터럽트가 HIGH 를 봤다 -> loop 가 잠그고 보고
bool           pwrLatched       = PWR_SENSE_ENABLED; // 정지 잠김 (CLR 로만 해제). 부팅 때 잠긴 채 시작
bool           pwrCycled        = false; // 잠긴 뒤 코일 OFF 를 한 번 봤는지 (A8 LOW 고착 검출용)
bool           pwrOffState      = false; // OFF 상태로 보고했는지 ("EVT PWR OFF" 중복 방지)
bool           pwrOnTiming      = false; // 코일 ON 안정 시간 재는 중
bool           pwrOk            = false; // 코일 ON 이 PWR_OK_STABLE_MS 이상 유지됨
uint32_t       pwrOnSince       = 0;     // 코일 ON 이 시작된 시각 (pwrOnTiming 일 때만 의미)

inline bool pwrSenseHigh() { return PWR_SENSE_ENABLED && (PINK & _BV(PK0)) != 0; }

// PWM 핀을 타이머 출력에서 떼어낸다 -> 핀은 PORT 값(LOW)으로 고정 = MDD10A 브레이크.
// OCR=0 만으로는 Fast PWM 에서 주기마다 1클럭(62.5ns) 펄스가 남는다. 이 방식은 펄스도 없다.
// 핀맵을 바꾸면 이 함수, pwmOutputsOn(), setupPwm3900Hz(), writePwm() 의 레지스터를 함께 바꾼다.
// 쉴드(HKX): D11 = OC1A, D12 = OC1B (Timer1) / D44 = OC5C, D45 = OC5B (Timer5).
void pwmOutputsOff() {
  TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0));
  TCCR5A &= ~(_BV(COM5B1) | _BV(COM5B0) | _BV(COM5C1) | _BV(COM5C0));
  OCR1A = OCR1B = 0; OCR5B = OCR5C = 0;
}

void pwmOutputsOn() {
  OCR1A = OCR1B = 0; OCR5B = OCR5C = 0;
  TCCR1A |= _BV(COM1A1) | _BV(COM1B1);
  TCCR5A |= _BV(COM5B1) | _BV(COM5C1);
}

ISR(PCINT2_vect) {
  if (pwrSenseHigh()) { pwmOutputsOff(); pwrTrip = true; }
}

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

// PWM 3.9kHz 타이머 설정: Timer1(OC1A=D11, OC1B=D12) + Timer5(OC5C=D44, OC5B=D45).
//   Fast PWM (mode 14), TOP=ICR=PWM_TOP, prescaler=1 -> 3.9kHz. 비반전(COMx1).
//   OC1C(D13, 보드 LED)와 OC5A(D46)는 켜지 않는다.
void setupPwm3900Hz() {
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = PWM_TOP; OCR1A = OCR1B = 0;
  TCCR5A = _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51);
  TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS50);
  ICR5 = PWM_TOP; OCR5B = OCR5C = 0;
}

// 듀티(0..255 스케일) -> 해당 모터 OCR(0..PWM_TOP) 직접 쓰기 (3.9kHz).
//   M1=D11=OC1A, M2=D44=OC5C, M3=D12=OC1B, M4=D45=OC5B
void writePwm(uint8_t i, uint16_t mag255) {
  uint16_t ocr = (uint16_t)((uint32_t)mag255 * PWM_TOP / 255);
  switch (i) {
    case 0: OCR1A = ocr; break;
    case 1: OCR5C = ocr; break;
    case 2: OCR1B = ocr; break;
    case 3: OCR5B = ocr; break;
  }
}

// 정지(듀티 0) 중에는 DIR 을 LOW 로 둔다 (2026-10-08). MDD10A 는 PWM LOW 면 방향과 무관하게 브레이크라
// 동작은 같고, MD 케이블이 뒤집혀 DIR 핀이 MDD10A GND 에 닿아 있어도 정지 중에는 출력 HIGH 가 GND 로 단락되지 않는다.
// 주의: 엔코더는 A상만 세므로 encDir 은 실제 회전 방향이 아니라 명령 부호다. 방향 판정은 눈으로 한다.
void applyWheel(uint8_t i, int duty) {
  bool fwd = (duty >= 0);
  int  mag = fwd ? duty : -duty;
  if (mag == 0) {
    writePwm(i, 0);                                   // PWM 먼저 0, 그다음 DIR LOW
    digitalWrite(DIR_PIN[i], LOW);
  } else {
    bool level = fwd ^ INVERT[i];                     // 전진여부 ^ 배선반전
    digitalWrite(DIR_PIN[i], level ? FWD_LEVEL : (FWD_LEVEL == HIGH ? LOW : HIGH));
    writePwm(i, (uint16_t)mag);                       // 3.9kHz OCR 쓰기
  }
  encDir[i] = fwd ? 1 : -1;
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

// 램프 출력 갱신. 점멸 램프는 모두 같은 위상으로 깜빡인다.
void applyLamps(uint32_t now) {
  bool blinkOn = ((now / BLINK_HALF_MS) % 2) == 0;
  for (uint8_t i = 0; i < NL; i++) {
    bool on = (lampMode[i] == 1) || (lampMode[i] == 2 && blinkOn);
    digitalWrite(LAMP_PIN[i], on ? HIGH : LOW);
  }
}

void printLamps() {
  Serial.print("LAMP");
  for (uint8_t i = 0; i < NL; i++) { Serial.print(' '); Serial.print(lampMode[i]); }
  Serial.println();
}

void printPwr() {
  if (!PWR_SENSE_ENABLED) { Serial.println("PWR DISABLED"); return; }
  Serial.print("PWR "); Serial.print(pwrOk ? 1 : 0);
  Serial.print(' ');    Serial.print(pwrLatched ? 1 : 0);
  Serial.print(' ');    Serial.println(pwrCycled ? 1 : 0);
}

// 동력 상실: 목표·현재 듀티 0, 출력 분리, 잠금. OFF 로 바뀔 때 1회 보고.
void latchPowerLoss() {
  pwmOutputsOff();
  for (uint8_t i = 0; i < N; i++) { target[i] = 0; current[i] = 0; }
  pwrLatched = true;
  pwrCycled  = true;                                   // 잠긴 뒤 코일 OFF 를 봤다
  pwrOnTiming = false;
  pwrOk = false;
  if (!pwrOffState) { pwrOffState = true; Serial.println("EVT PWR OFF"); }
}

// 동력 감시 갱신 (loop 에서 매번). millis() 는 한 번만 읽는다.
void updatePower() {
  if (!PWR_SENSE_ENABLED) return;
  if (pwrTrip || pwrSenseHigh()) {                     // 짧은 글리치라도 인터럽트가 봤으면 OFF 로 친다
    pwrTrip = false;
    latchPowerLoss();
    return;
  }
  uint32_t nowMs = millis();
  if (!pwrOnTiming) { pwrOnTiming = true; pwrOnSince = nowMs; }
  if (!pwrOk && nowMs - pwrOnSince >= PWR_OK_STABLE_MS) {
    pwrOk = true;
    pwrOffState = false;
    if (pwrLatched && pwrCycled) Serial.println("EVT PWR READY");   // CLR 가능. 잠김은 CLR 까지 유지
  }
}

void handle(const String &cmd) {
  // "P " 는 두 모터 명령. "PWR" 조회와 구별하려고 공백까지 본다.
  if (pwrLatched && (cmd.startsWith("V") || cmd.startsWith("M") || cmd.startsWith("P "))) {
    Serial.println("ERR PWR");                        // 잠김 중엔 구동 명령 거부 (데드맨도 갱신 안 함)
    return;
  }
  if (cmd == "PWR") {
    printPwr();
  } else if (cmd == "CLR") {
    bool cleared = false;
    if (pwrLatched && pwrCycled && pwrOk) {
      doEstop();                                      // 목표·현재 0 에서 다시 시작
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {             // 확인과 출력 복귀 사이에 인터럽트가 끼지 않게
        if (!pwrSenseHigh() && !pwrTrip) { pwrLatched = false; pwrCycled = false; pwmOutputsOn(); cleared = true; }
      }
    }
    if (cleared || !pwrLatched) {
      if (cleared) lastCmd = millis();
      Serial.println("OK CLR");
    } else if (!pwrCycled) {
      Serial.println("ERR PWR CYCLE");
    } else if (pwrSenseHigh() || pwrTrip || !pwrOnTiming) {
      Serial.println("ERR PWR OFF");
    } else {
      Serial.println("ERR PWR WAIT");
    }
  } else if (cmd.startsWith("V")) {
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
  } else if (cmd.startsWith("P ")) {
    // 두 모터 테스트: "P <i> <j> <pct>" (i != j, 1..4). 나머지 두 모터는 0.
    int a = 0, b = 0, pct = 0;
    if (sscanf(cmd.c_str() + 1, "%d %d %d", &a, &b, &pct) == 3 &&
        a >= 1 && a <= N && b >= 1 && b <= N && a != b) {
      int duty = clampi((int)(clampi(pct, -100, 100) * ((float)MAX_DUTY / 100.0f)), -MAX_DUTY, MAX_DUTY);
      for (uint8_t i = 0; i < N; i++) target[i] = 0;
      target[a - 1] = duty;
      target[b - 1] = duty;
      lastCmd = millis();
      Serial.print("OK P "); Serial.print(a); Serial.print(' '); Serial.println(b);
    } else {
      Serial.println("ERR P");
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
  } else if (cmd == "L" || cmd.startsWith("L ")) {
    // 램프는 모터 안전과 무관하므로 데드맨(lastCmd)을 갱신하지 않는다.
    int n = -1, s = -1;
    if (cmd == "L") {
      printLamps();
    } else if (sscanf(cmd.c_str() + 1, "%d %d", &n, &s) == 2 &&
               n >= 0 && n <= NL && s >= 0 && s <= 2) {
      for (uint8_t i = 0; i < NL; i++) {
        if (n == 0 || n == i + 1) lampMode[i] = (uint8_t)s;
      }
      applyLamps(millis());
      printLamps();
    } else {
      Serial.println("ERR L");
    }
  } else if (cmd == "ID") {
    // ★보드 식별: Mega 가 2개(주행/팔)라 COM 번호가 뒤바뀔 수 있음.
    //   파이썬이 접속 시 "ID" 를 보내 이 응답으로 어느 보드인지 확인/자동배정한다.
    Serial.println("MECANUM");
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
  for (uint8_t i = 0; i < NL; i++) {              // 램프: LOW 를 먼저 쓰고 출력 전환 -> 부팅 중 켜짐 방지
    digitalWrite(LAMP_PIN[i], LOW);
    pinMode(LAMP_PIN[i], OUTPUT);
  }
  setupPwm3900Hz();                               // PWM 3.9kHz (요구사항), 듀티 0 으로 시작
  pinMode(PWR_SENSE_PIN, INPUT_PULLUP);           // 선이 빠지면 HIGH = 정지 쪽
  if (PWR_SENSE_ENABLED) {
    pwmOutputsOff();                              // 부팅은 잠긴 상태 -> 출력 분리
    PCMSK2 |= _BV(PCINT16);                       // A8 핀변화 인터럽트
    PCIFR  |= _BV(PCIF2);
    PCICR  |= _BV(PCIE2);
  }
  Serial.begin(115200);
  lastTick = lastCmd = millis();
  Serial.println("READY");
  if (!PWR_SENSE_ENABLED) Serial.println("WARN PWR SENSE OFF");
  else printPwr();                                // 부팅 상태 1줄: "PWR <ok> 1 <cycled>"
}

// 비블로킹 줄 읽기: '\n' 을 받은 줄만 실행한다(잘린 줄이 명령으로 실행되지 않게). 너무 긴 줄은 버린다.
char     lineBuf[48];
uint8_t  lineLen  = 0;
bool     lineOver = false;

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (!lineOver) {
        lineBuf[lineLen] = '\0';
        String cmd(lineBuf);
        cmd.trim();                                   // '\r' 도 여기서 빠진다
        if (cmd.length() > 0) handle(cmd);
      }
      lineLen = 0; lineOver = false;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineOver = true;
    }
    updatePower();                                    // 긴 입력 중에도 동력 감시는 계속
  }
}

void loop() {
  // 동력 감시: 인터럽트가 이미 출력을 끊었다. 여기서는 잠그고 보고한다(폴링은 인터럽트 누락 대비).
  updatePower();
  pollSerial();

  uint32_t now = millis();
  // 데드맨: 명령이 끊기면 목표를 0 으로(완만 정지). PC 다운/케이블 분리 보호.
  if (now - lastCmd > CMD_TIMEOUT_MS) { for (uint8_t i = 0; i < N; i++) target[i] = 0; }

  if (now - lastTick >= TICK_MS) { lastTick = now; updateMotors(); applyLamps(now); }
}
