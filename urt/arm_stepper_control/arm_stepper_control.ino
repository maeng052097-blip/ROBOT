/*
  arm_stepper_control.ino  --  ST-NK1748E 스텝모터 3개: 모터별 '각도 이동' + '각도 측정' 기본세팅
  대상: 별도(새) 아두이노 하나. AccelStepper 라이브러리 필요.
  기능: (1) 각 모터를 개별로 (2) 원하는 각도만큼 돌리고 (3) 지금 몇 도인지 읽기 + 속도/영점.

  ===== 필요한 라이브러리 (한 번만) =====
    Arduino IDE -> 스케치 -> 라이브러리 포함하기 -> 라이브러리 관리 -> "AccelStepper"(Mike McCauley) 설치.

  ===== 제어 (시리얼 모니터, 115200. 각 명령은 치고 Enter) =====
    1 <deg>   : 모터1 을 <deg>도 만큼 이동(상대). 예) '1 90' = +90도,  '1 -45' = -45도
    2 <deg>   : 모터2 이동          3 <deg> : 모터3 이동
    p         : 3개 모터 현재 각도 출력(측정)
    z         : 지금 위치를 0도로(전체 영점)
    v <dps>   : 속도 설정(도/초). 예) 'v 150'
    s         : 즉시 정지(감속). 이동 중 취소.
    -> 여러 모터에 명령을 연달아 주면 동시에 움직입니다(각자 목표까지).

  ===== 배선 (앞 테스트와 동일, 공통애노드 5V) =====
    모터1: PU-(노랑)->핀2, DIR-(주황)->핀3 | 모터2: ->핀4,핀5 | 모터3: ->핀6,핀7
    PU+(초록)/DIR+(보라) 전부 -> 아두이노 5V | V+(빨강)->24V, V-(검정)->배터리- | EN/AL 미사용
    아두이노-24V 접지 공유 불필요(옵토절연).

  ===== ★ 각도 정확도의 핵심: STEPS_PER_REV 가 DIP 마이크로스텝과 반드시 같아야 함 =====
    지금 DIP = SW1,2,3 ON -> 1000 step/rev -> 아래 STEPS_PER_REV=1000. (0.36도/스텝)
    더 세밀하게: DIP 를 3200(SW1 ON,SW2 OFF,SW3 ON) 으로 바꾸고 STEPS_PER_REV=3200 (0.1125도/스텝).
    ※ DIP 와 이 값이 다르면 '90도' 명령이 엉뚱한 각도로 돕니다.

  ===== 측정에 대한 정직한 설명 =====
    여기서 보고하는 각도 = '보낸 스텝 수'로 계산한 '명령 각도'입니다. 이 모터는 closed-loop(내장
    엔코더 1000CPR)라 드라이버가 그 명령 위치까지 실제로 맞춰갑니다(못 맞추면 AL 경보). 즉 경보만
    없으면 명령각도 ~= 실제각도. 드라이버 내부 엔코더 실측값은 10핀에 안 나와서(별도 시리얼 필요)
    아두이노가 직접은 못 읽습니다. 실측검증이 필요하면 나중에 AL(경보)선을 읽어 스텝손실을 감지.
*/
#include <AccelStepper.h>

const long  STEPS_PER_REV = 1000;                      // ★DIP 와 일치시킬 것 (SW1,2,3 ON = 1000)
const float STEPS_PER_DEG = STEPS_PER_REV / 360.0;

// DRIVER 모드 = (STEP핀, DIR핀). 모터1/2/3
AccelStepper m1(AccelStepper::DRIVER, 2, 3);
AccelStepper m2(AccelStepper::DRIVER, 4, 5);
AccelStepper m3(AccelStepper::DRIVER, 6, 7);
AccelStepper* M[3] = {&m1, &m2, &m3};

float speed_dps = 900;    // 속도(도/초)
float accel_dps2 = 2000.0;   // 가속(도/초^2)

char buf[24];
byte blen = 0;

void applySpeedAccel() {
  for (int i = 0; i < 3; i++) {
    M[i]->setMaxSpeed(speed_dps * STEPS_PER_DEG);
    M[i]->setAcceleration(accel_dps2 * STEPS_PER_DEG);
  }
}

void reportPos() {
  for (int i = 0; i < 3; i++) {
    float deg = M[i]->currentPosition() / STEPS_PER_DEG;
    Serial.print("M"); Serial.print(i + 1); Serial.print("=");
    Serial.print(deg, 1); Serial.print("deg  ");
  }
  Serial.println();
}

void handleLine(char* s) {
  while (*s == ' ') s++;                 // 앞 공백 제거
  char c = *s;
  if (c == 'p' || c == 'P') { reportPos(); return; }
  if (c == 'z' || c == 'Z') { for (int i = 0; i < 3; i++) M[i]->setCurrentPosition(0);
                              Serial.println("ZERO (0 deg)"); return; }
  if (c == 's' || c == 'S') { for (int i = 0; i < 3; i++) M[i]->stop();
                              Serial.println("STOP"); return; }
  if (c == 'v' || c == 'V') { float v = atof(s + 1); if (v > 0) { speed_dps = v; applySpeedAccel(); }
                              Serial.print("speed="); Serial.print(speed_dps); Serial.println(" deg/s"); return; }
  if (c >= '1' && c <= '3') {
    int idx = c - '1';
    float deg = atof(s + 1);             // 명령 문자 뒤의 숫자(각도)
    long steps = (long)(deg * STEPS_PER_DEG + (deg >= 0 ? 0.5f : -0.5f));
    M[idx]->move(steps);                 // 상대 이동(현재 목표에 더함)
    Serial.print("M"); Serial.print(idx + 1); Serial.print(" move ");
    Serial.print(deg, 1); Serial.println(" deg");
    return;
  }
  Serial.println("?");
}

void setup() {
  for (int i = 0; i < 3; i++) {
    // 공통애노드(active-LOW): STEP 핀을 반전 -> 유휴 HIGH(LED off), 스텝시 LOW 펄스(LED on)
    M[i]->setPinsInverted(false, true, false);   // (dir, step, enable). 방향 반대면 첫 인자 true 로
    M[i]->setMinPulseWidth(5);                    // 옵토가 확실히 인식하도록 5us
  }
  applySpeedAccel();
  Serial.begin(115200);
  Serial.println("READY  (1 90 / 2 -45 / 3 360 = move deg | p=pos | z=zero | v 150=speed | s=stop)");
}

void loop() {
  // ---- 시리얼 라인 수집(비블로킹) ----
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (blen > 0) { buf[blen] = 0; handleLine(buf); blen = 0; }
    } else if (blen < (byte)sizeof(buf) - 1) {
      buf[blen++] = ch;
    }
  }
  // ---- 3개 모터 비블로킹 구동(각자 목표까지 가감속) ----
  m1.run();
  m2.run();
  m3.run();
}
