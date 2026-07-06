"""mecanum_drive_test.py — 메카넘 합성 주행(V vx vy w) 테스트. 하드웨어 필요(Mega+MDD10A x2, COM3).

목적: 개별 모터(M 명령) 확인이 끝난 뒤, '합성 믹싱'이 맞는지 본다.
      전진/후진(vx), 좌우 평행이동(vy, strafe), 좌우 회전(w) 을 각각 순수하게 보내
      (1) 로봇이 의도한 방향으로 가는지 (2) 4바퀴 엔코더 부호 패턴이 맞는지 확인.
      펌웨어 믹싱: FL=vx+vy+w  FR=vx-vy-w  RL=vx-vy+w  RR=vx+vy-w (POS_TO_MOTOR/STRAFE_SIGN 적용)

★안전: 첫 시험은 반드시 '바퀴를 들고'(받침대 위). 로봇이 스스로 움직인다.
        이상 시 즉시 x(ESTOP). 데드맨 1.5s 라 이 도구가 0.15s 마다 재전송해 유지한다.

명령 규약: "V <vx> <vy> <w>" 각 -100~100(%). vx+전진, vy+우평행, w+우회전(CW).

실행: py -3.13 tests/mecanum_drive_test.py            (기본 COM3)
      py -3.13 tests/mecanum_drive_test.py --port COM3 --speed 25

키(각 키 = 그 방향 '순수' 모션, 나머지 축은 0):
  w / s : 전진 / 후진            (vx +/-)
  d / a : 우 / 좌 평행이동       (vy +/-)
  l / j : 우회전 / 좌회전         (w  +/-)
  ] / [ : 속도 +5% / -5%  (5~100, 기본 25)
  space : STOP(완만 정지)        x : ESTOP(즉시)
  z     : 엔코더 카운트 리셋      q/ESC : 종료(자동 STOP)

검증 기준(엔코더 부호 패턴, 바퀴 들고):
  전진(w키): 4바퀴 모두 같은 부호(예 전부 +).   후진: 전부 반대.
  우회전(l키): 좌측(FL,RL) 과 우측(FR,RR) 이 서로 반대 부호.
  우평행(d키): 대각선 쌍이 같은 부호(메카넘 롤러 특성). 로봇이 오른쪽으로.
  -> 로봇 실제 움직임 + 부호 패턴이 위와 다르면 config 의 INVERT/POS_TO_MOTOR/STRAFE_SIGN 조정.
"""
import argparse
import sys
import time
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

try:    # 콘솔(cp949)에 없는 문자를 print 해도 앱이 안 죽게
    sys.stdout.reconfigure(errors="replace")
    sys.stderr.reconfigure(errors="replace")
except Exception:
    pass

from common import config

RESEND_S = 0.15     # 명령 재전송 주기(데드맨 1.5s 유지)
ENC_S = 0.5


def open_serial(port, baud):
    import serial
    ser = serial.Serial(port, baud, timeout=0.05)
    time.sleep(2.0)              # Mega 리셋 대기
    ser.reset_input_buffer()
    return ser


def send(ser, line):
    ser.write((line + "\n").encode("ascii"))


def read_lines(ser):
    out = []
    try:
        while ser.in_waiting:
            ln = ser.readline().decode("ascii", "replace").strip()
            if ln:
                out.append(ln)
    except Exception:
        pass
    return out


def motion_label(vx, vy, w):
    if vx == 0 and vy == 0 and w == 0:
        return "정지"
    parts = []
    if vx > 0: parts.append("전진")
    elif vx < 0: parts.append("후진")
    if vy > 0: parts.append("우평행")
    elif vy < 0: parts.append("좌평행")
    if w > 0: parts.append("우회전")
    elif w < 0: parts.append("좌회전")
    return "+".join(parts)


def main():
    ap = argparse.ArgumentParser(description="메카넘 합성 주행(V) 테스트 - 바퀴 들고")
    ap.add_argument("--port", default=config.MOTOR_PORT, help=f"모터 포트(기본 {config.MOTOR_PORT})")
    ap.add_argument("--baud", type=int, default=config.MOTOR_BAUDRATE)
    ap.add_argument("--speed", type=int, default=25, help="속도 %% (5~100, 기본 25)")
    args = ap.parse_args()

    try:
        import msvcrt
    except ImportError:
        print("[실패] Windows 콘솔 전용 도구입니다(msvcrt).")
        return

    try:
        ser = open_serial(args.port, args.baud)
    except Exception as e:
        print(f"[실패] 시리얼 열기 {args.port}: {e}")
        print("  - 다른 도구가 포트 점유 중인지, 케이블/포트번호 확인 (tests/check_devices.py)")
        return

    print(f"[연결] {args.port} @ {args.baud}")
    print("*** 안전: 바퀴 들고 시험. space=STOP, x=ESTOP ***")
    print("키: w/s 전후 | a/d 좌우평행 | j/l 좌우회전 | ]/[ 속도 | space STOP | x ESTOP | q 종료")

    speed = max(5, min(100, args.speed))
    vx = vy = w = 0
    last_send = 0.0
    last_enc = 0.0
    enc_prev = None
    running = True

    def status():
        print(f"[명령] V {vx} {vy} {w}   속도 {speed}%   -> {motion_label(vx, vy, w)}")

    def set_motion(nx, ny, nw):
        nonlocal vx, vy, w
        vx, vy, w = nx, ny, nw
        send(ser, f"V {vx} {vy} {w}")

    status()
    try:
        while running:
            now = time.time()

            while msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'q', b'Q', b'\x1b'):
                    running = False
                elif ch in (b'w', b'W'):
                    set_motion(speed, 0, 0); last_send = now; status()
                elif ch in (b's', b'S'):
                    set_motion(-speed, 0, 0); last_send = now; status()
                elif ch in (b'd', b'D'):
                    set_motion(0, speed, 0); last_send = now; status()
                elif ch in (b'a', b'A'):
                    set_motion(0, -speed, 0); last_send = now; status()
                elif ch in (b'l', b'L'):
                    set_motion(0, 0, speed); last_send = now; status()
                elif ch in (b'j', b'J'):
                    set_motion(0, 0, -speed); last_send = now; status()
                elif ch == b']':
                    speed = min(100, speed + 5); status()
                elif ch == b'[':
                    speed = max(5, speed - 5); status()
                elif ch == b' ':
                    set_motion(0, 0, 0); enc_prev = None; status()
                elif ch in (b'x', b'X'):
                    vx = vy = w = 0; send(ser, "ESTOP"); enc_prev = None
                    print("[ESTOP] 즉시 정지"); status()
                elif ch in (b'z', b'Z'):
                    send(ser, "ENCRESET"); enc_prev = None

            # 구동 유지(데드맨)
            moving = not (vx == 0 and vy == 0 and w == 0)
            if moving and now - last_send >= RESEND_S:
                send(ser, f"V {vx} {vy} {w}")
                last_send = now

            # 엔코더(구동 중에만) - 4바퀴 부호 패턴으로 믹싱 검증
            if moving and now - last_enc >= ENC_S:
                send(ser, "ENC")
                last_enc = now

            for ln in read_lines(ser):
                if ln.startswith("ENC"):
                    try:
                        c = [int(x) for x in ln.split()[1:5]]
                    except ValueError:
                        continue
                    if enc_prev is not None:
                        dt = now - enc_prev[0]
                        if dt > 0:
                            r = [(c[i] - enc_prev[1][i]) / dt for i in range(4)]
                            print(f"ENC rate  M1 {r[0]:+.0f}  M2 {r[1]:+.0f}  "
                                  f"M3 {r[2]:+.0f}  M4 {r[3]:+.0f}  cnt/s")
                    enc_prev = (now, c)
                elif ln.startswith("ERR") or ln.startswith("UNK"):
                    print("[펌웨어]", ln)

            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            send(ser, "STOP"); time.sleep(0.3); send(ser, "ESTOP")
            ser.close()
        except Exception:
            pass
        print("종료(STOP 전송됨)")


if __name__ == "__main__":
    main()
