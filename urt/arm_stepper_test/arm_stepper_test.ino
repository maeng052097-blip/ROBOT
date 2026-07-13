/*
  arm_stepper_test.ino  --  ST-NK1748E closed-loop 스텝모터 3개: 시리얼로 '동시/개별 시작·정지' 제어
  대상: 별도(새) 아두이노 하나. Uno/Nano/Mega 아무거나(디지털 6핀).
  목적: 모터 3개를 한꺼번에(또는 하나씩) 원할 때 돌리고 멈추기. 추가 부품 없이 시리얼 모니터로.

  ===== 제어 (Arduino IDE 시리얼 모니터, 보드레이트 115200) =====
    g : 3개 모두 시작            s (또는 스페이스) : 3개 모두 정지
    1 / 2 / 3 : 모터 1/2/3 개별 켬/끔(토글)   -> 하나만/두개만 돌리기도 가능
    f / r : 방향 정/역(전체)      + / - : 속도 빠르게/느리게(전체)      ? : 상태 보기
    -> 시리얼 모니터 입력칸에 글자 치고 Enter(전송).

  ===== 배선 (드라이버 3개 동일, 공통애노드 5V) =====
    전원(24V):  24V+ -> 각 드라이버 V+(빨강, pin1) / 24V- -> 각 드라이버 V-(검정, pin2)
                (3개 드라이버 V- 는 서로 묶어 배터리(-)로)
    신호(5V):   아두이노 5V -> 각 드라이버 PU+(초록,3) 와 DIR+(보라,5)   (+입력 전부 5V 로 공통)
                모터1: PU-(노랑) -> 핀2 ,  DIR-(주황) -> 핀3
                모터2: PU-(노랑) -> 핀4 ,  DIR-(주황) -> 핀5
                모터3: PU-(노랑) -> 핀6 ,  DIR-(주황) -> 핀7
    EN(흰/파)=미연결(기본 구동) / AL(갈/회)=미사용 / 아두이노-24V 접지 공유 불필요(옵토절연).

  ===== DIP (드라이버 3개 전부 동일하게!) =====
    SW4 = OFF (스텝/방향 모드, ★필수) / SW1,SW2,SW3 = ON,ON,ON (1000 step/rev) / SW5 = 방향(아무거나)

  ※ 첫 시험은 축에 아무것도 안 달고. 반대로 도는 모터는 f/r 로. 안 돌면 그 드라이버 DIP/전류설정 확인.
*/

const int N = 3;
const int STEP_PIN[N] = {2, 4, 6};    // 각 드라이버 PU-(노랑): 모터1=2, 모터2=4, 모터3=6
const int DIR_PIN[N]  = {3, 5, 7};    // 각 드라이버 DIR-(주황): 모터1=3, 모터2=5, 모터3=7

int          dir_level = HIGH;        // 방향(공통애노드 DIR- 핀 레벨). f/r 로 바뀜(전체 공통)
unsigned int half_us   = 500;         // 스텝 반주기(us). 작을수록 빠름(500 -> 약 1회전/초 @1000step/rev)
bool         run[N]    = {false, false, false};  // 모터별 시작/정지 (전원 넣으면 전부 정지)

void applyDir() {
  for (int i = 0; i < N; i++) digitalWrite(DIR_PIN[i], dir_level);
}

void printStatus() {
  Serial.print("M1="); Serial.print(run[0] ? "ON" : "off");
  Serial.print(" M2="); Serial.print(run[1] ? "ON" : "off");
  Serial.print(" M3="); Serial.print(run[2] ? "ON" : "off");
  Serial.print("  dir="); Serial.print(dir_level == HIGH ? "fwd" : "rev");
  Serial.print("  half_us="); Serial.println(half_us);
}

void setup() {
  for (int i = 0; i < N; i++) {
    pinMode(STEP_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    digitalWrite(STEP_PIN[i], HIGH);  // 유휴 = HIGH(공통애노드: LED off)
  }
  applyDir();
  Serial.begin(115200);
  Serial.println("READY  (g=start-all  s=stop-all  1/2/3=toggle motor  f/r=dir  +/-=speed  ?=status)");
}

void loop() {
  // ---- 시리얼 명령 (한 글자씩) ----
  while (Serial.available()) {
    char c = Serial.read();
    if      (c == 'g' || c == 'G')                { for (int i = 0; i < N; i++) run[i] = true;  Serial.println("RUN all");  }
    else if (c == 's' || c == 'S' || c == ' ')    { for (int i = 0; i < N; i++) run[i] = false; Serial.println("STOP all"); }
    else if (c == '1')                            { run[0] = !run[0]; printStatus(); }
    else if (c == '2')                            { run[1] = !run[1]; printStatus(); }
    else if (c == '3')                            { run[2] = !run[2]; printStatus(); }
    else if (c == 'f' || c == 'F')                { dir_level = HIGH; applyDir(); Serial.println("DIR fwd"); }
    else if (c == 'r' || c == 'R')                { dir_level = LOW;  applyDir(); Serial.println("DIR rev"); }
    else if (c == '+')                            { if (half_us > 100) half_us -= 50; printStatus(); }
    else if (c == '-')                            { half_us += 50; printStatus(); }
    else if (c == '?')                            { printStatus(); }
    // 그 외(줄바꿈 등)는 무시
  }

  // ---- 구동: 켜진 모터들만 동시에 스텝(같은 속도로 lockstep). 하나도 안 켜졌으면 스킵 ----
  if (run[0] || run[1] || run[2]) {
    for (int i = 0; i < N; i++) if (run[i]) digitalWrite(STEP_PIN[i], LOW);
    delayMicroseconds(half_us);
    for (int i = 0; i < N; i++) if (run[i]) digitalWrite(STEP_PIN[i], HIGH);
    delayMicroseconds(half_us);
  }
}
