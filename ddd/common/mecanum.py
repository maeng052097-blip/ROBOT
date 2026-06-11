"""메카넘 4휠 믹싱 (순수 함수).

차체 속도 명령 (vx, vy, w) 을 4개 바퀴 듀티로 변환한다. 부호 약속:
  vx = + 전진 / vy = + 우측 strafe / w = + 우회전(CW, 시계방향)
바퀴 배치(기본 가정): FL=앞왼, FR=앞오, RL=뒤왼, RR=뒤오. (X-롤러 메카넘)
  FL = vx + vy + w
  FR = vx - vy - w
  RL = vx - vy + w
  RR = vx + vy - w
검산) 전진 vx>0 -> 4휠 +.  우strafe vy>0 -> FL+,FR-,RL-,RR+.  우회전 w>0 -> 좌측(FL,RL)+,우측(FR,RR)-.

이 파일은 펌웨어(urt/mecanum_motor/src/main.cpp)의 mixSet() 과 '동일한 식'을 쓴다
(스펙·단위테스트용 단일 출처). 식을 바꾸면 양쪽을 같이 고쳐야 한다.
"""


def mix(vx, vy, w, max_pct=100):
    """(vx, vy, w) -> (FL, FR, RL, RR) 퍼센트. 합성이 max_pct 를 넘으면 비례 축소(방향 보존)."""
    fl = vx + vy + w
    fr = vx - vy - w
    rl = vx - vy + w
    rr = vx + vy - w
    m = max(abs(fl), abs(fr), abs(rl), abs(rr))
    if m > max_pct:
        s = max_pct / float(m)
        fl, fr, rl, rr = fl * s, fr * s, rl * s, rr * s
    return (int(round(fl)), int(round(fr)), int(round(rl)), int(round(rr)))
