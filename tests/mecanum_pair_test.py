"""mecanum_pair_test.py — 바퀴 2개씩 구동 시험 (하드웨어 필요: Mega + MDD10A x2, 펌웨어 urt/mecanum_motor). 2026-10-10

목적: 4바퀴를 한꺼번에 돌리기 전에 두 바퀴씩 짝지어 방향과 엔코더를 확인한다.
      M1 = 앞왼, M2 = 앞오, M3 = 뒤왼, M4 = 뒤오 (라이다 쪽이 뒤).
펌웨어 명령: "P <i> <j> <pct>" (두 모터만 구동, 나머지 0). 펌웨어 INVERT 가 적용되므로 + 면 두 바퀴 모두 앞으로 굴러야 한다.

조작 (누르고 있는 동안만 돈다. 손을 떼면 STOP = 완만 정지):
  1 왼쪽  (M1 앞왼 + M3 뒤왼)        2 오른쪽 (M2 앞오 + M4 뒤오)
  3 앞    (M1 앞왼 + M2 앞오)        4 뒤     (M3 뒤왼 + M4 뒤오)
  5 대각  (M1 앞왼 + M4 뒤오)        6 대각   (M2 앞오 + M3 뒤왼)
  위 방향키 = 앞으로(+), 아래 방향키 = 뒤로(-)
  +/- (Backspace 옆 키) = 속도 5% 증감 (정지 중에만, 5~40%)
  Space = 정지, X 또는 Esc = 비상정지(ESTOP) 후 종료
  O = 펌웨어 동력 잠김 해제(CLR). PCB-C 를 단 뒤 비상정지 해제 + START 다음에 사람이 직접(자동 전송 없음)

안전:
  - 반드시 바퀴를 든 상태에서. 펌웨어에 스톨 감지가 없으므로 오래 누르고 있지 않는다.
  - 이 프로그램을 실행한 창이 맨 앞에 있을 때만 키를 받는다(다른 창으로 가면 정지).
  - 시작 후 모든 키에서 한 번 손을 뗀 다음부터 입력을 받는다.
  - 정지/모터 전환/아두이노 리셋('READY') 뒤에는 방향키를 새로 눌러야 돈다(누른 채로 있으면 다시 돌지 않음).
  - 반복 루프가 0.4초 넘게 멈추면 ESTOP 후 종료(예: 실행 중 터미널 글자를 마우스로 클릭/드래그).
  - 펌웨어 식별(ID = MECANUM)이 안 되면 구동하지 않고 종료한다(받은 수동 펌웨어 등 다른 펌웨어 보호).
판정: 엔코더 부호는 펌웨어가 A상만 세서 '명령 부호'다(실제 회전 방향 아님). 방향은 눈으로 본다.
  엔코더로는 '고른 두 바퀴만 돌았는지'와 '두 바퀴 속도가 비슷한지'를 본다. 기록: data/motor_logs/pair_test_*.log

실행: py -3.13 tests/mecanum_pair_test.py --port COM4 --speed 20
"""
import argparse
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

try:    # 콘솔(cp949)에 없는 문자를 print 해도 앱이 안 죽게
    sys.stdout.reconfigure(errors="replace")
    sys.stderr.reconfigure(errors="replace")
except Exception:
    pass

from common import config

PAIRS = {
    '1': (1, 3, '왼쪽 (M1 앞왼 + M3 뒤왼)'),
    '2': (2, 4, '오른쪽 (M2 앞오 + M4 뒤오)'),
    '3': (1, 2, '앞 (M1 앞왼 + M2 앞오)'),
    '4': (3, 4, '뒤 (M3 뒤왼 + M4 뒤오)'),
    '5': (1, 4, '대각 (M1 앞왼 + M4 뒤오)'),
    '6': (2, 3, '대각 (M2 앞오 + M3 뒤왼)'),
}
VK = {'up': 0x26, 'down': 0x28, 'space': 0x20, 'x': 0x58, 'esc': 0x1B,
      '1': 0x31, '2': 0x32, '3': 0x33, '4': 0x34, '5': 0x35, '6': 0x36,
      '+': 0xBB, '-': 0xBD, 'o': 0x4F}
TICK_S = 0.025      # 키 읽기 주기
RESEND_S = 0.1      # 구동 명령 재전송(펌웨어 데드맨 1.5s 유지)
ENC_S = 0.3         # 구동 중 엔코더 조회
PRINT_S = 1.0       # 상태 출력
STALL_LOOP_S = 0.4  # 이보다 오래 멈추면 ESTOP 후 종료
MAX_SPEED = 40      # 이 시험의 상한(%)
LOG_DIR = pathlib.Path(__file__).resolve().parent.parent / "data" / "motor_logs"   # git 제외

_log = None


def say(*parts, flush=True):
    """화면 출력 + 기록 파일(있으면)."""
    text = " ".join(str(p) for p in parts)
    print(text, flush=flush)
    log_only(text)


def log_only(text):
    if _log is not None:
        _log.write(f"{time.monotonic():.3f} {text}\n")
        _log.flush()


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


def identify(ser, wait_s=1.5):
    """ID / PWR 를 묻고 응답 줄을 돌려준다. ID 가 MECANUM 이 아니면 구동하지 않는다."""
    send(ser, "ID")
    send(ser, "PWR")
    lines, end = [], time.time() + wait_s
    while time.time() < end:
        lines += read_lines(ser)
        time.sleep(0.02)
    return lines


def main():
    global _log
    ap = argparse.ArgumentParser(description="바퀴 2개씩 구동 시험 - 바퀴 들고")
    ap.add_argument("--port", default=config.MOTOR_PORT, help=f"모터 포트(기본 {config.MOTOR_PORT})")
    ap.add_argument("--baud", type=int, default=config.MOTOR_BAUDRATE)
    ap.add_argument("--speed", type=int, default=20, help=f"속도 %% (5~{MAX_SPEED}, 기본 20)")
    args = ap.parse_args()

    if sys.platform != "win32":
        say("[실패] Windows 전용 도구입니다(키를 누르고 있는 동안만 구동).")
        return
    import ctypes
    import msvcrt
    user32 = ctypes.WinDLL("user32", use_last_error=True)
    user32.GetForegroundWindow.restype = ctypes.c_void_p
    user32.GetAsyncKeyState.argtypes = [ctypes.c_int]
    user32.GetAsyncKeyState.restype = ctypes.c_short
    foreground = user32.GetForegroundWindow()
    if not foreground:
        say("[실패] 실행 창을 확인하지 못했습니다.")
        return

    try:
        ser = open_serial(args.port, args.baud)
    except Exception as e:
        say(f"[실패] 시리얼 열기 {args.port}: {e}")
        say("  - 다른 도구가 포트 점유 중인지, 포트 번호 확인 (py -3.13 -m serial.tools.list_ports -v)")
        return

    lines = identify(ser)
    for ln in lines:
        say("[펌웨어]", ln)
    if "MECANUM" not in lines:
        say("[중단] 펌웨어 ID 가 MECANUM 이 아닙니다 -> urt/mecanum_motor 를 업로드한 보드에서만 씁니다. 구동하지 않고 종료.")
        ser.close()
        return

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / ("pair_test_" + time.strftime("%Y%m%d_%H%M%S") + ".log")
    _log = open(log_path, "w", encoding="utf-8")
    say("[기록]", log_path)
    for ln in lines:
        log_only("[펌웨어] " + ln)

    speed = max(5, min(MAX_SPEED, args.speed))
    pair = '1'
    direction = 0            # 현재 구동 방향: 0 정지, +1 앞, -1 뒤
    released = False         # 시작 후 모든 키를 한 번 놓았는지
    prev_keys = set()
    last_tick = last_send = last_enc = last_print = 0.0
    enc = None
    run_base = None          # 출발 순간 엔코더 값 -> '이번 구동 누적' 계산 (2026-10-10: 정지 중에도 ENC 를 읽어 이전 구동의 잔여 회전이 섞이지 않게)

    def status(msg=""):
        i, j, name = PAIRS[pair]
        d = {0: "정지", 1: "앞으로(+)", -1: "뒤로(-)"}[direction]
        say(f"[짝 {pair}] {name} | {d} {speed}% {msg}", flush=True)

    def stop():
        nonlocal direction
        if direction != 0:
            send(ser, "STOP")
            direction = 0

    say(f"[연결] {args.port} @ {args.baud}")
    say("*** 바퀴를 든 상태에서. 누르고 있는 동안만 돈다. Space=정지, X/Esc=비상정지 후 종료 ***")
    say("짝: 1 왼쪽, 2 오른쪽, 3 앞, 4 뒤, 5 대각(앞왼+뒤오), 6 대각(앞오+뒤왼) | 위/아래 방향키 = 앞/뒤 | +/- 속도(정지 중)")
    say("시작: 모든 키에서 손을 뗀 뒤 XT60 을 꽂고, 짝 번호를 고른 다음 방향키를 누르고 있으세요.")
    status()
    try:
        while True:
            now = time.monotonic()
            if last_tick and now - last_tick > STALL_LOOP_S:
                send(ser, "ESTOP")
                say("[중단] 반복 루프가 0.4초 넘게 멈춤 -> ESTOP 후 종료 (실행 중 터미널 글자 클릭/드래그 금지)")
                break
            last_tick = now

            focused = user32.GetForegroundWindow() == foreground
            keys = {k for k, v in VK.items() if user32.GetAsyncKeyState(v) & 0x8000} if focused else set()
            while msvcrt.kbhit():           # 콘솔 입력 버퍼를 비워 종료 뒤 프롬프트에 글자가 남지 않게
                msvcrt.getch()
            if not released:
                released = not keys
                keys = set()
            pressed = keys - prev_keys
            prev_keys = keys

            if 'x' in keys or 'esc' in keys:
                send(ser, "ESTOP")
                direction = 0
                say("[ESTOP] 비상정지 후 종료")
                break
            if 'space' in pressed:
                stop()
                status("(Space)")
            if 'o' in pressed:
                if direction == 0:
                    send(ser, "CLR")
                    say("[PWR] CLR 전송 (사람이 직접)")
                else:
                    say("CLR 은 정지 상태에서만 보냅니다")
            for k in PAIRS:
                if k in pressed:
                    stop()
                    pair = k
                    status("(짝 선택)")
            for k, delta in (('+', 5), ('-', -5)):
                if k in pressed:
                    if direction == 0:
                        speed = max(5, min(MAX_SPEED, speed + delta))
                        status("(속도 변경)")
                    else:
                        say("속도 변경은 정지 중에만 됩니다 (방향키에서 손을 떼고 다시)")

            want = 0
            if 'up' in keys and 'down' not in keys:
                want = 1
            elif 'down' in keys and 'up' not in keys:
                want = -1
            if want == 0:
                if direction != 0:
                    stop()
                    status("(손 뗌)")
                    if enc is not None and run_base is not None:
                        say(f"  이번 구동 누적 M1~M4 {[enc[k] - run_base[k] for k in range(4)]} (손 뗀 순간, 이후 관성 회전 제외)")
            elif direction == 0:
                # 새로 누른 방향키로만 출발한다(정지·리셋 뒤 누른 채로 있던 키로는 다시 돌지 않음).
                if ('up' in pressed and want == 1) or ('down' in pressed and want == -1):
                    i, j, _ = PAIRS[pair]
                    direction = want
                    send(ser, f"P {i} {j} {direction * speed}")
                    last_send = now
                    run_base = list(enc) if enc is not None else None
                    last_print = now
                    status("(출발)")
            elif want != direction:
                stop()                      # 반대 방향은 손을 뗐다가 다시
                status("(방향 바꿈 -> 손을 떼고 다시)")
            elif now - last_send >= RESEND_S:
                i, j, _ = PAIRS[pair]
                send(ser, f"P {i} {j} {direction * speed}")
                last_send = now

            if now - last_enc >= ENC_S:          # 정지 중에도 읽는다(ENC 는 데드맨을 갱신하지 않음)
                send(ser, "ENC")
                last_enc = now

            for ln in read_lines(ser):
                if ln.startswith("ENC"):
                    try:
                        enc = [int(x) for x in ln.split()[1:5]]
                        log_only(f"ENC {enc} dir={direction} pair={pair} speed={speed}")
                    except ValueError:
                        pass
                elif ln == "READY":
                    direction = 0
                    say("[펌웨어] READY - 아두이노가 리셋됨 -> 정지. 원인 확인 뒤 방향키를 새로 누를 것")
                elif ln.startswith("UNK P"):
                    direction = 0
                    say("[펌웨어]", ln, "-> 이 펌웨어에는 P(두 모터) 명령이 없습니다. urt/mecanum_motor 를 다시 업로드")
                elif ln.startswith(("ERR", "UNK", "EVT", "WARN")) or ln == "OK CLR":
                    if ln.startswith("EVT PWR OFF"):
                        direction = 0
                    say("[펌웨어]", ln)

            if direction != 0 and enc is not None and now - last_print >= PRINT_S:
                change = [enc[k] - run_base[k] for k in range(4)] if run_base else None
                say(f"  ENC M1~M4 {enc}  (이번 구동 누적 {change})", flush=True)
                last_print = now

            time.sleep(TICK_S)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            send(ser, "STOP"); time.sleep(0.3); send(ser, "ESTOP")
            ser.close()
        except Exception:
            pass
        while msvcrt.kbhit():
            msvcrt.getch()
        say("종료(STOP/ESTOP 전송됨). 바퀴가 선 것을 확인하고 XT60 을 빼세요.")
        if _log is not None:
            _log.close()


if __name__ == "__main__":
    main()
