/*
 * Herkulex DRS-0601 - 5초간 연속 회전 (속도제어 모드)
 * 라이브러리: cesarvandevelde/HerkulexServo  (의존성: CircularBuffer by AgileWare)
 * 보드: Arduino Mega 2560 (하드웨어 Serial1 사용: TX1=18, RX1=19)
 *
 * [중요/검증된 사실]
 * - 이 라이브러리는 공식적으로 DRS-0101/0201 전용. 0601은 "속도(연속회전) 모드"에서
 *   안전하게 동작(위치 해상도와 무관). 위치 모드는 0601의 2048스텝 때문에 각도가 어긋남.
 * - 제어모드 변경은 토크 OFF 상태에서만 가능 → off → 모드전환 → on 순서를 지킬 것.
 * - setSpeed 범위: -1023 ~ 1023 (음수 = 역방향).
 * - "5초"는 명령 playtime(최대 ~2.86s)이 아니라 millis() 타이머로 제어.
 *
 * [하드웨어 주의]
 * - DRS-0601 전원: 9.5~14.8VDC (12V 권장). 0101/0201용 7.4V를 그대로 쓰지 말 것.
 * - GND는 전원과 아두이노를 반드시 공통으로 연결.
 * - 통신 기본 속도 115200. (LED가 빨강으로 깜빡이면 보레이트 불일치 의심)
 */

#include <SoftwareSerial.h>   // 라이브러리가 요구하는 헤더(Mega에선 직접 안 써도 포함)
#include <CircularBuffer.h>   // 외부 의존성
#include <HerkulexServo.h>

// ===== 사용자 설정 =====
const uint8_t       SERVO_ID  = 0xFE;   // 실제 서보 ID를 알면 그 값으로! 0xFE = 브로드캐스트(전체)
const int16_t       SPIN_SPEED = 400;   // -1023~1023. 음수면 반대방향. 처음엔 작게 시작 권장
const unsigned long RUN_MS    = 5000;   // 회전 시간 (ms)

HerkulexServoBus herkulex_bus(Serial1);          // 통신 버스 (Mega 하드웨어 Serial1)
HerkulexServo    servo(herkulex_bus, SERVO_ID);  // 서보 인스턴스

unsigned long start_time = 0;
bool stopped = false;

void setup() {
  Serial.begin(115200);    // USB 디버그용 시리얼 모니터
  Serial1.begin(115200);   // 허큘렉스 통신 (기본 보레이트)
  delay(300);

  // 제어모드 변경은 반드시 토크 OFF 상태에서
  servo.setTorqueOff();
  delay(50);
  servo.enableSpeedControlMode();   // 연속회전(속도제어) 모드
  delay(50);
  servo.setTorqueOn();              // 모터에 힘 인가
  delay(50);

  // 회전 시작
  servo.setSpeed(SPIN_SPEED, 0, HerkulexLed::Green);
  start_time = millis();
  Serial.println(F("Spin start (5s)"));
}

void loop() {
  herkulex_bus.update();   // 매 루프마다 호출 — 수신 패킷 처리(필수)

  if (!stopped && (millis() - start_time >= RUN_MS)) {
    servo.setSpeed(0, 0, HerkulexLed::Off);   // 정지
    delay(50);
    servo.setTorqueOff();                     // 힘 풀기(선택). 정지 상태 유지하려면 이 줄 삭제
    stopped = true;
    Serial.println(F("Stopped after 5s"));
  }
}
