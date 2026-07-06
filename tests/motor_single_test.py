"""motor_single_test.py — 모터 1개씩 정/역방향 구동 테스트 (하드웨어 필요: Mega+MDD10A, COM3).

목적: 수리/교체 모터 장착 후 각 모터(PWM 5/6/7/8)를 하나씩 돌려
      (1) 정방향/역방향 모두 도는지 (2) 엔코더 카운트가 움직이는지(=실회전, 스톨 감지)
      (3) 드라이버 채널(M2 등)이 살아있는지 확인한다.

펌웨어 명령(urt/mecanum_motor): "M <i> <pct>" (i=1..4=M1..M4=PWM5..8, pct -100..100, 부호=방향)
  데드맨 1.5s -> 이 도구가 0.2s 마다 명령을 재전송해 유지한다. 키를 떼도 계속 돌므로
  space(STOP)/e(ESTOP) 로 멈춘다.

실행: py -3.13 tests/motor_single_test.py            (기본 COM3)
      py -3.13 tests/motor_single_test.py --port COM3 --speed 25

키:
  1 2 3 4 : 모터 선택 (1=M1/PWM5, 2=M2/PWM6, 3=M3/PWM7, 4=M4/PWM8)
  f       : 정방향(+) 구동      r : 역방향(-) 구동
  ] / [   : 속도 +5% / -5%  (5~100%, 기본 25%)
  space   : STOP(완만 정지)     e : ESTOP(즉시 정지)
  z       : 엔코더 카운트 리셋
  q / ESC : 종료(자동 STOP)

★안전: 반드시 '바퀴를 들고'(로봇을 받침대 위에) 시험할 것. 처음엔 낮은 속도(25%)로.
  회전 중 이상음/발열/LED 이상 -> 즉시 e(ESTOP) 후 전원 차단.
판정 기준:
  - f/r 양방향 모두 회전 + 엔코더 카운트 증가/감소 = 정상
  - PWM 명령은 OK 인데 카운트 정지/모터 무회전 = 스톨 또는 드라이버 채널/배선 불량
  - 한 방향만 회전 = DIR 배선/드라이버 반쪽(하프브리지) 불량 의심
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

RESEND_S = 0.2      # 명령 재전송 주기(데드맨 1.5s 유지용)
ENC_S = 0.6         # 엔코더 조회 주기
PWM_OF = {1: 5, 2: 6, 3: 7, 4: 8}   # 모터번호 -> PWM 핀(표시용)


def open_serial(port, baud):
    import serial
    ser = serial.Serial(port, baud, timeout=0.05)
    time.sleep(2.0)              # Mega 리셋 대기(READY 수신)
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


def main():
    ap = argparse.ArgumentParser(description="모터 1개씩 정/역방향 구동 테스트(엔코더 확인 포함)")
    ap.add_argument("--port", default=config.MOTOR_PORT, help=f"모터 시리얼 포트(기본 {config.MOTOR_PORT})")
    ap.add_argument("--baud", type=int, default=config.MOTOR_BAUDRATE)
    ap.add_argument("--speed", type=int, default=25, help="시작 속도 %% (5~100, 기본 25)")
    args = ap.parse_args()

    try:
        import msvcrt                      # Windows 콘솔 키 입력
    except ImportError:
        print("[실패] Windows 콘솔 전용 도구입니다(msvcrt).")
        return

    try:
        ser = open_serial(args.port, args.baud)
    except Exception as e:
        print(f"[실패] 시리얼 열기 {args.port}: {e}")
        print("  - 다른 도구(track 등)가 포트 점유 중인지, 케이블/포트번호 확인 (tests/check_devices.py)")
        return

    print(f"[연결] {args.port} @ {args.baud}")
    print("*** 안전: 바퀴를 들고(받침대 위) 시험하세요. space=STOP, e=ESTOP ***")
    print("키: 1-4 모터선택 | f 정방향 | r 역방향 | ]/[ 속도+-5% | z 엔코더리셋 | q 종료")

    motor = 1
    speed = max(5, min(100, args.speed))
    pct = 0                     # 현재 구동 명령(부호=방향, 0=정지)
    last_send = 0.0
    last_enc = 0.0
    enc_prev = None             # (시각, [c1..c4]) — 카운트 변화율 표시용
    running = True

    def status():
        d = {0: "정지", 1: "정방향", -1: "역방향"}[0 if pct == 0 else (1 if pct > 0 else -1)]
        print(f"[상태] 모터 M{motor}(PWM{PWM_OF[motor]})  속도 {speed}%  {d}")

    status()
    try:
        while running:
            now = time.time()

            # --- 키 입력 ---
            while msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'q', b'Q', b'\x1b'):
                    running = False
                elif ch in (b'1', b'2', b'3', b'4'):
                    if pct != 0:
                        send(ser, "STOP"); pct = 0     # 모터 전환 시 우선 정지
                    motor = int(ch); enc_prev = None; status()
                elif ch in (b'f', b'F'):
                    pct = speed; send(ser, f"M {motor} {pct}"); last_send = now; status()
                elif ch in (b'r', b'R'):
                    pct = -speed; send(ser, f"M {motor} {pct}"); last_send = now; status()
                elif ch == b']':
                    speed = min(100, speed + 5)
                    if pct != 0: pct = speed if pct > 0 else -speed
                    status()
                elif ch == b'[':
                    speed = max(5, speed - 5)
                    if pct != 0: pct = speed if pct > 0 else -speed
                    status()
                elif ch == b' ':
                    pct = 0; send(ser, "STOP"); enc_prev = None; status()
                elif ch in (b'e', b'E'):
                    pct = 0; send(ser, "ESTOP"); enc_prev = None; print("[ESTOP] 즉시 정지"); status()
                elif ch in (b'z', b'Z'):
                    send(ser, "ENCRESET"); enc_prev = None

            # --- 구동 유지(데드맨) ---
            if pct != 0 and now - last_send >= RESEND_S:
                send(ser, f"M {motor} {pct}")
                last_send = now

            # --- 엔코더 확인 (구동 중에만; 정지 상태에선 폴링·출력 없이 조용) ---
            if pct != 0 and now - last_enc >= ENC_S:
                send(ser, "ENC")
                last_enc = now

            # --- 수신 처리 ---
            for ln in read_lines(ser):
                if ln.startswith("ENC"):
                    try:
                        c = [int(x) for x in ln.split()[1:5]]
                    except ValueError:
                        continue
                    if enc_prev is not None:
                        dt = now - enc_prev[0]
                        if dt > 0:
                            rate = (c[motor - 1] - enc_prev[1][motor - 1]) / dt
                            stall = "  <- 무회전(스톨/채널 의심!)" if (pct != 0 and abs(rate) < 5) else ""
                            print(f"ENC M{motor}={c[motor - 1]}  ({rate:+.0f} cnt/s){stall}   전체 {c}")
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
