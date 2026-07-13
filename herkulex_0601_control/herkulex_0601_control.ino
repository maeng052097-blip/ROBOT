/*
 * herkulex_0601_control.ino  --  Herkulex DRS-0601: 정/역 회전 + 현재 각도 + 회전량 측정
 * 기반: herkulex_0601_spin_5s.ino  (라이브러리 cesarvandevelde/HerkulexServo, 의존 CircularBuffer)
 * 보드: Arduino Mega 2560, 하드웨어 Serial1 (TX1=18, RX1=19). 전원 12V, GND 공통.
 *
 * ===== 시리얼 모니터(115200) 명령 =====
 *   setid <n>   : ★내가 원하는 ID 를 서보에 심기(EEP+RAM). 반드시 '서보 1개만' 연결한 상태에서!
 *                 서보2개는 A만 연결->setid 1, B만 연결->setid 2, 그다음 둘 다 연결.
 *   scan        : 응답하는 실제 서보 ID 자동 탐색 후 지정
 *   id <n>      : 대상 서보 ID 수동 지정. ★위치를 읽으려면 실제 ID 필요(브로드캐스트 0xFE로는 못 읽음)
 *   f <0-1023>  : 정방향 회전(속도).  예) 'f 400'
 *   r <0-1023>  : 역방향 회전.        예) 'r 400'
 *   s           : 정지(속도 0, 토크 유지)
 *   on / off    : 토크 켜기 / 끄기
 *   p           : 현재 위치(raw) + 각도(추정) + 시작 이후 회전량(누적)
 *   z           : 회전량/기준 0으로(영점)
 *   stat        : 서보 알람(에러) 레지스터 읽기 -> 빨강LED 원인 확인(과부하/전압/온도 등)
 *   clr         : 알람(에러) 클리어 시도
 *   ack         : 서보 응답정책(AckPolicy)=1 강제. scan 이 '응답없음'일 때 먼저 시도(재배선 불필요)
 *   loop        : 루프백 자가진단. 18(TX1)-19(RX1) 점퍼로 잇고 실행 -> 아두이노 수신부 정상여부
 *   cal         : 회전량 보정 안내(1바퀴 counts 측정)
 *
 * ===== ★각도/회전량 정확도 =====
 *   - getPosition()의 raw 값은 항상 정확합니다(그대로 신뢰).
 *   - 도(deg) 환산은 COUNTS_PER_REV 에 의존. 라이브러리 getPosition()이 위치를 10비트(0~1023)로
 *     마스킹해 읽으므로 기본 1024. (서보 물리분해능이 더 크면 마스크가 반바퀴에서 되감겨 다회전
 *     누적이 깨질 수 있음 -> cal 로 1바퀴 counts 실측 확인.)
 *   - 서보 ID 를 모르면: id 253(공장기본) 먼저 시도. 'f 300' 후 'p' 를 여러 번 쳐서 raw 가
 *     '변하면' 그 ID가 맞습니다. 안 변하면 id 0, id 1 ... 로 바꿔 시도.
 */
#include <SoftwareSerial.h>   // 라이브러리 요구 헤더
#include <CircularBuffer.h>
#include <HerkulexServo.h>
#include <string.h>           // strncmp / strpbrk (명령 파싱)

uint8_t servo_id       = 1;    // ★실제 서보 ID (읽기엔 브로드캐스트 254 불가). 모르면 253부터
long    COUNTS_PER_REV = 1024;   // ★getPosition()이 10비트(0~1023)로 마스킹해 읽음 -> 기본 1024. cal 로 실측

HerkulexServoBus bus(Serial1);
HerkulexServo*   servo = nullptr;

long total_counts = 0;           // 누적 회전(raw counts, wrap 보정)
int  last_raw = -1;
bool have_last = false;
bool track_pos = false;          // 위치 주기샘플 on/off. 읽기(RX) 확인되면 true (안되면 stall 방지 위해 off)
unsigned long sample_t = 0;

char buf[24];
byte blen = 0;

void makeServo() {
  if (servo) delete servo;
  servo = new HerkulexServo(bus, servo_id);
}

int readRaw() {
  return servo ? (int)servo->getPosition() : -1;
}

// 특정 ID가 응답하는지 '빠르게' 확인: STAT 보내고 짧은 창(12ms)만 대기.
// (라이브러리 기본 sendPacketAndReadResponse 는 없는 ID마다 6회x100ms=600ms 대기 -> 스캔이 2~3분)
bool servoResponds(uint8_t testId) {
  bus.update();
  HerkulexPacket junk;
  while (bus.getPacket(junk)) {}                  // 잔여 응답 비우기
  bus.sendPacket(testId, HerkulexCommand::Stat);  // STAT 요청(응답 유도)
  unsigned long t0 = millis();
  while (millis() - t0 < 12) {                     // 12ms 안에 유효 응답 오면 존재
    bus.update();
    HerkulexPacket resp;
    if (bus.getPacket(resp) && resp.id == testId && resp.error == HerkulexPacketError::None)
      return true;
  }
  return false;
}

// 0~253 훑어 응답하는 첫 ID 찾기(254=브로드캐스트는 응답 안 하므로 제외). 없으면 255.
uint8_t scanId() {
  for (int i = 0; i <= 253; i++) {
    if (servoResponds((uint8_t)i)) return (uint8_t)i;
  }
  return 255;
}

void enterSpeedMode() {
  servo->setTorqueOff();              delay(30);
  servo->enableSpeedControlMode();    delay(30);   // 모드 변경은 토크 OFF 상태에서만
}

void spin(int spd) {                  // spd 양수=정방향, 음수=역방향
  servo->setTorqueOn();               delay(5);
  servo->setSpeed(spd, 0, HerkulexLed::Green);
}

void reportPos() {
  int raw = readRaw();
  float ang = raw * 360.0 / COUNTS_PER_REV;             // 1회전 내 각도(추정)
  float turned = total_counts * 360.0 / COUNTS_PER_REV; // 시작 이후 총 회전(추정)
  Serial.print(F("raw=")); Serial.print(raw);
  Serial.print(F("  angle~")); Serial.print(ang, 1); Serial.print(F("deg"));
  Serial.print(F("  turned~")); Serial.print(turned, 1); Serial.print(F("deg ("));
  Serial.print(turned / 360.0, 2); Serial.println(F(" rev)"));
}

// ★내가 원하는 ID 를 서보에 '심기'. 반드시 버스에 서보가 '1개'일 때만 실행!
//   (여러 개면 전부 같은 ID -> 통신 파탄). EEP(영구)+RAM(즉시) 둘 다 써서 바로 적용.
//   쓰기는 브로드캐스트로 확실히 먹으므로 현재 ID 를 몰라도 됨(수신 안돼도 OK).
void assignId(uint8_t new_id) {
  HerkulexServo bc(bus, HERKULEX_BROADCAST_ID);   // 0xFE = 버스의 (유일한) 서보 지목
  bc.setTorqueOff();                               delay(10);   // EEP 쓰기는 토크 OFF 권장
  bc.writeEep(HerkulexEepRegister::ID, new_id);    delay(10);   // EEPROM(영구 저장)
  bc.writeRam(HerkulexRamRegister::ID, new_id);    delay(10);   // RAM(즉시 반영)
  servo_id = new_id; makeServo(); have_last = false;
  enterSpeedMode();                                             // 새 ID 로 속도모드 재진입
  bool ok = servoResponds(servo_id);                            // ★진짜 응답확인(STAT). getID()는 캐시라 가짜
  track_pos = ok;
  Serial.print(F("setid -> ")); Serial.print(new_id);
  Serial.println(ok ? F("  [OK] 서보가 응답함 -> p 로 각도 읽힘")
                    : F("  [주의] 응답 없음. 회전(TX)은 되지만 각도(RX) 안됨: 서보TXD->핀19/전원 확인"));
}

void printStatus() {   // ★서보 알람(에러) 레지스터를 읽어 빨강LED 원인 표시. 읽기(RX)가 돼야 유효.
  HerkulexStatusError err; HerkulexStatusDetail det;
  servo->getStatus(err, det);
  uint8_t e = (uint8_t)err;
  Serial.print(F("STAT err=0x")); Serial.print(e, HEX);
  Serial.print(F(" detail=0x")); Serial.println((uint8_t)det, HEX);
  if (e == 0) { Serial.println(F("  에러비트 0 (진짜 정상이거나, RX무응답으로 0). 빨강이 계속이면 RX부터.")); return; }
  if (e & 0x01) Serial.println(F("  [ERR] 입력전압 한계 (12V 전원/전류 부족 의심)"));
  if (e & 0x02) Serial.println(F("  [ERR] POT(위치) 한계 초과"));
  if (e & 0x04) Serial.println(F("  [ERR] 온도 한계 초과"));
  if (e & 0x08) Serial.println(F("  [ERR] 잘못된 패킷 수신"));
  if (e & 0x10) Serial.println(F("  [ERR] 과부하(Overload) - 축 부하/기구 간섭"));
  if (e & 0x20) Serial.println(F("  [ERR] 드라이버 결함"));
  if (e & 0x40) Serial.println(F("  [ERR] EEP 손상"));
}

// 서보가 응답(ACK)하도록 강제: AckPolicy=1(READ/STAT 에 응답). 브로드캐스트 쓰기라 수신 안돼도 먹음.
// 서보가 '응답 안함(0)'으로 설정돼 있었으면 이걸로 살아나 scan 이 성공함.
void forceAck() {
  HerkulexServo bc(bus, HERKULEX_BROADCAST_ID);
  bc.writeEep(HerkulexEepRegister::AckPolicy, 1);  delay(10);   // 영구
  bc.writeRam(HerkulexRamRegister::AckPolicy, 1);  delay(10);   // 즉시
  Serial.println(F("AckPolicy=1 설정(브로드캐스트). 이제 'scan' 다시 -> 응답하면 이게 원인이었음."));
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(300);
  makeServo();
  enterSpeedMode();                   // 속도(연속회전) 모드로 준비 (토크는 OFF로 시작)
  Serial.println(F("READY  setid<n>|scan|id<n>|f<spd>|r<spd>|s|on|off|p|z|stat|clr|ack|loop|cal"));
  Serial.println(F("  ID 부여: 서보 1개만 연결 -> 'setid 1' (2개면 하나씩). 그래야 각도(p) 읽힘."));
}

long argnum(char *s) {                // 문자열 어디에 있든 첫 숫자(부호 포함)를 읽음
  while (*s && !(*s == '-' || (*s >= '0' && *s <= '9'))) s++;
  return atoi(s);
}

void handleLine(char *s) {
  while (*s == ' ') s++;
  if (strncmp(s, "setid", 5) == 0) {  // 내가 원하는 ID 심기 (서보 1개만 연결한 상태에서!)
    if (!strpbrk(s + 5, "0123456789")) { Serial.println(F("사용법: setid <0-253>  (버스에 서보 1개만!)")); return; }
    int n = (int)argnum(s);
    if (n < 0) n = 0; if (n > 253) n = 253;
    assignId((uint8_t)n);
    return;
  }
  if (strncmp(s, "scan", 4) == 0) {   // 응답하는 실제 ID 자동 탐색 ('s'=정지보다 먼저 체크)
    Serial.println(F("scanning 0..253 (몇 초 소요)..."));
    uint8_t found = scanId();
    if (found == 255) {
      Serial.println(F("  응답 서보 없음 -> 배선(TX/RX)/전원/보레이트(115200) 확인"));
    } else {
      servo_id = found; makeServo(); have_last = false; track_pos = true;
      Serial.print(F("  found ID = ")); Serial.print(found);
      Serial.println(F("  -> 이제 이 ID로 f/r/p 동작(각도 읽기 가능)"));
    }
    return;
  }
  if (strncmp(s, "stat", 4) == 0) { printStatus(); return; }   // 빨강LED 원인(에러비트) 읽기
  if (strncmp(s, "clr", 3) == 0) {                              // 알람(에러) 클리어 시도
    servo->writeRam(HerkulexRamRegister::StatusError, 0);
    servo->writeRam(HerkulexRamRegister::StatusDetail, 0);
    Serial.println(F("알람 클리어 시도. 원인(전원/부하/RX)이 남아있으면 다시 뜸")); return;
  }
  if (strncmp(s, "ack", 3) == 0) { forceAck(); return; }        // 서보 응답정책 강제 ON
  if (strncmp(s, "loop", 4) == 0) {                             // 루프백: 18-19 점퍼 후 실행
    while (Serial1.available()) Serial1.read();                 // 수신버퍼 비우기
    uint8_t probe[3] = {0xA5, 0x5A, 0x3C};
    Serial1.write(probe, 3); Serial1.flush();
    delay(20);
    int cnt = 0; Serial.print(F("loopback rx:"));
    while (Serial1.available()) { Serial.print(' '); Serial.print(Serial1.read(), HEX); cnt++; }
    Serial.println();
    Serial.println(cnt >= 3 ? F("  [OK] 에코 옴 -> 아두이노 TX1/RX1 정상. 문제는 서보 TXD/배선 쪽")
                            : F("  [무에코] 18-19 점퍼했다면 RX1(19) 문제. (점퍼 안했으면 서보TXD 무신호)"));
    return;
  }
  char c = *s;
  int  n = (int)argnum(s);            // 명령 뒤 숫자(f 400, id 253 등 위치 무관)
  if (c == 'f') { spin(constrain(n, 0, 1023)); Serial.print(F("FWD ")); Serial.println(n); }
  else if (c == 'r') { spin(-constrain(n, 0, 1023)); Serial.print(F("REV ")); Serial.println(n); }
  else if (c == 's') { servo->setSpeed(0, 0, HerkulexLed::Off); Serial.println(F("STOP")); }
  else if (c == 'o' && s[1] == 'n') { servo->setTorqueOn(); Serial.println(F("torque ON")); }
  else if (c == 'o' && s[1] == 'f') { servo->setTorqueOff(); Serial.println(F("torque OFF")); }
  else if (c == 'p') { reportPos(); }
  else if (c == 'z') { total_counts = 0; have_last = false; Serial.println(F("ZERO")); }
  else if (c == 'i' && s[1] == 'd') {
    servo_id = (uint8_t)n; makeServo(); have_last = false;
    track_pos = servoResponds(servo_id);
    Serial.print(F("ID = ")); Serial.print(servo_id);
    Serial.println(track_pos ? F("  [OK 응답]") : F("  [무응답]"));
  }
  else if (c == 'c') {   // cal
    Serial.println(F("[보정] 1) z 로 영점 -> 2) 축을 정확히 1바퀴 회전 -> 3) p 의 turned 가 아니라"));
    Serial.println(F("        total_counts 변화가 곧 COUNTS_PER_REV. 그 값을 코드 상수에 넣기."));
    Serial.print(F("  현재 raw=")); Serial.print(readRaw());
    Serial.print(F("  total_counts=")); Serial.print(total_counts);
    Serial.print(F("  현재 CPR=")); Serial.println(COUNTS_PER_REV);
  }
  else { Serial.println(F("? setid<n>/scan/id<n>/f/r/s/on/off/p/z/stat/clr/ack/loop/cal")); }
}

void loop() {
  bus.update();                       // 수신 처리(필수)

  // 시리얼 명령 수집(비블로킹)
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (blen > 0) { buf[blen] = 0; handleLine(buf); blen = 0; }
    } else if (blen < (byte)sizeof(buf) - 1) {
      buf[blen++] = ch;
    }
  }

  // 위치 주기 샘플링 -> 회전량 누적(wrap 보정). 25ms 마다. (읽기 확인된 track_pos 일 때만)
  if (track_pos && millis() - sample_t >= 25) {
    sample_t = millis();
    int raw = readRaw();
    if (raw >= 0) {
      if (have_last) {
        int d = raw - last_raw;
        if (d >  COUNTS_PER_REV / 2) d -= COUNTS_PER_REV;   // 아래로 wrap
        else if (d < -COUNTS_PER_REV / 2) d += COUNTS_PER_REV; // 위로 wrap
        total_counts += d;
      }
      last_raw = raw;
      have_last = true;
    }
  }
}
