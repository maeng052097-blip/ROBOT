"""mecanum_drive_test.py — 메카넘 4바퀴 합성 주행(V vx vy w) 시험. 하드웨어 필요(Mega + MDD10A x2, 펌웨어 urt/mecanum_motor).

2026-10-10 개정: 키 배치를 track 도구(track_and_approach*.py)와 맞추고, 누르고 있는 동안만 돌게 바꿨다.
  또 '바퀴가 완전히 멈춘 것을 확인한 뒤에만' 다른 방향(또는 같은 방향)으로 다시 출발한다.

키 (누르고 있는 동안만 돈다. 손을 떼면 STOP = 완만 정지):
  W / S : 전진 / 후진              (vx +/-)
  A / D : 제자리 좌회전 / 우회전   (w  -/+, vx = vy = 0 -> 로봇 중심을 축으로 그 자리에서 돈다)
  Z / C : 좌 / 우 평행이동          (vy -/+, 메카넘 전용)
  +/- (Backspace 옆 키) : 속도 5% 증감 (정지 중에만, 5~40%, 기본 20)
  Space : 정지
  X / E / Esc : 비상정지(ESTOP) 후 종료
  O : 펌웨어 동력 잠김 해제(CLR). PCB-C 를 단 뒤 비상정지 해제 + START 다음에 사람이 직접 누른다(자동 전송 없음).

정지 확인 규칙:
  - 손을 떼거나 Space 를 누르면 STOP 을 보내고, 엔코더 네 칸이 0.3초 동안 변하지 않을 때까지(최대 2초) '정지 확인 중'이다.
  - 그동안 누른 키는 무시한다. '정지 확인' 이 뜬 뒤 키를 새로 눌러야 출발한다(누른 채로 있던 키로는 출발하지 않음).
  - 두 개 이상의 이동 키를 동시에 누르면 정지한다.

안전:
  - 첫 시험은 반드시 바퀴를 든 상태에서. 펌웨어에 스톨 감지가 없으므로 오래 누르고 있지 않는다.
  - 이 프로그램을 실행한 창이 맨 앞에 있을 때만 키를 받는다(다른 창으로 가면 정지).
  - 시작 후 모든 키에서 한 번 손을 뗀 다음부터 입력을 받는다.
  - 반복 루프가 0.4초 넘게 멈추면 ESTOP 후 종료(예: 실행 중 터미널 글자를 마우스로 클릭/드래그).
  - 펌웨어 식별(ID = MECANUM)이 안 되면 구동하지 않고 종료한다.
  - 아두이노가 리셋되면('READY') 정지하고, 키를 새로 눌러야 다시 돈다.

판정 (바퀴 든 상태, 방향은 눈으로 - 엔코더 부호는 펌웨어가 A상만 세서 '명령 부호'다):
  W: 네 바퀴 모두 앞으로          S: 네 바퀴 모두 뒤로
  D: 왼쪽(앞왼, 뒤왼) 앞 / 오른쪽(앞오, 뒤오) 뒤 -> 바닥에서는 제자리 시계 방향 회전        A: 그 반대
  C: 앞왼, 뒤오 앞 / 앞오, 뒤왼 뒤                      Z: 그 반대
  펌웨어 믹싱: FL=vx+vy+w  FR=vx-vy-w  RL=vx-vy+w  RR=vx+vy-w (POS_TO_MOTOR/INVERT/STRAFE_SIGN 적용)
  실제로 오른쪽으로 평행이동하는지는 바닥 최저속에서만 보인다(그때 반대면 main.cpp 의 STRAFE_SIGN = -1).
  다르면 urt/mecanum_motor/src/main.cpp 의 INVERT/POS_TO_MOTOR/STRAFE_SIGN 을 고치고 XT60 을 뺀 상태로 재업로드.
기록: data/motor_logs/drive_test_*.log

실행: py -3.13 tests/mecanum_drive_test.py --port COM4 --speed 20
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

# 키 -> (vx, vy, w) 부호, 이름. track_and_approach_single.py 의 수동 키와 같은 배치.
MOTIONS = {
    'w': ((1, 0, 0), '전진'),
    's': ((-1, 0, 0), '후진'),
    'a': ((0, 0, -1), '제자리 좌회전'),
    'd': ((0, 0, 1), '제자리 우회전'),
    'z': ((0, -1, 0), '좌 평행이동'),
    'c': ((0, 1, 0), '우 평행이동'),
}
VK = {'w': 0x57, 's': 0x53, 'a': 0x41, 'd': 0x44, 'z': 0x5A, 'c': 0x43,
      'space': 0x20, 'x': 0x58, 'e': 0x45, 'esc': 0x1B, '+': 0xBB, '-': 0xBD, 'o': 0x4F}
TICK_S = 0.025          # 키 읽기 주기
RESEND_S = 0.1          # 구동 명령 재전송(펌웨어 데드맨 1.5s 유지)
ENC_S = 0.3             # 엔코더 조회(구동/대기 중)
ENC_STOPPING_S = 0.1    # 정지 확인 중 엔코더 조회
STILL_S = 0.3           # 이 시간 동안 네 칸이 그대로면 '정지 확인'
STOP_TIMEOUT_S = 2.0    # 엔코더로 확인이 안 돼도 이 시간이 지나면 정지로 본다(듀티는 이미 0)
PRINT_S = 1.0           # 구동 중 상태 출력
STALL_LOOP_S = 0.4      # 이보다 오래 멈추면 ESTOP 후 종료
MAX_SPEED = 40          # 이 시험의 상한(%)
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
    ap = argparse.ArgumentParser(description="메카넘 4바퀴 합성 주행(V) 시험 - 바퀴 들고")
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
    log_path = LOG_DIR / ("drive_test_" + time.strftime("%Y%m%d_%H%M%S") + ".log")
    _log = open(log_path, "w", encoding="utf-8")
    say("[기록]", log_path)
    for ln in lines:
        log_only("[펌웨어] " + ln)

    speed = max(5, min(MAX_SPEED, args.speed))
    state = "ready"          # ready: 출발 가능 / moving: 구동 중 / stopping: 정지 확인 중
    motion = None            # 구동 중인 키 ('w' 등)
    released = False         # 시작 후 모든 키를 한 번 놓았는지
    prev_keys = set()
    last_tick = last_send = last_enc = last_print = 0.0
    stop_at = 0.0            # STOP 을 보낸 시각
    enc = None
    enc_changed_at = 0.0     # 엔코더 값이 마지막으로 바뀐 시각
    run_base = None          # 출발 순간 엔코더 값

    def command(key):
        (sx, sy, sw), _ = MOTIONS[key]
        return f"V {sx * speed} {sy * speed} {sw * speed}"

    def status(msg=""):
        name = MOTIONS[motion][1] if motion else "-"
        label = {"ready": "정지(출발 가능)", "moving": f"{name} 중", "stopping": "정지 확인 중"}[state]
        say(f"[{label}] 속도 {speed}% {msg}", flush=True)

    def begin_stop(reason):
        nonlocal state, stop_at
        if state == "moving":
            send(ser, "STOP")
            stop_at = time.monotonic()
            state = "stopping"
            if enc is not None and run_base is not None:
                say(f"  이번 구동 누적 M1~M4 {[enc[k] - run_base[k] for k in range(4)]} (손 뗀 순간)")
            status(reason)

    say(f"[연결] {args.port} @ {args.baud}")
    say("*** 바퀴를 든 상태에서. 누르고 있는 동안만 돈다. Space=정지, X/E/Esc=비상정지 후 종료 ***")
    say("키: W 전진, S 후진, A 제자리 좌회전, D 제자리 우회전, Z 좌 평행이동, C 우 평행이동 | +/- 속도(정지 중)")
    say("다른 방향은 바퀴가 멈춘 것('정지 확인')을 본 뒤 키를 새로 눌러야 출발한다.")
    say("시작: 모든 키에서 손을 뗀 뒤 XT60 을 꽂고, 이동 키를 누르고 있으세요.")
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

            if keys & {'x', 'e', 'esc'}:
                send(ser, "ESTOP")
                state = "ready"
                say("[ESTOP] 비상정지 후 종료")
                break
            if 'space' in pressed:
                begin_stop("(Space)")
            if 'o' in pressed:
                if state == "ready":
                    send(ser, "CLR")
                    say("[PWR] CLR 전송 (사람이 직접)")
                else:
                    say("CLR 은 '정지(출발 가능)' 상태에서만 보냅니다")
            for k, delta in (('+', 5), ('-', -5)):
                if k in pressed:
                    if state == "ready":
                        speed = max(5, min(MAX_SPEED, speed + delta))
                        status("(속도 변경)")
                    else:
                        say("속도 변경은 '정지(출발 가능)' 상태에서만 됩니다")

            held = keys & set(MOTIONS)
            if state == "moving":
                if not focused:
                    begin_stop("(창 전환)")
                elif held != {motion}:
                    # 손을 뗐거나, 다른 이동 키를 같이 누름 -> 정지부터
                    begin_stop("(손 뗌)" if not held else "(다른 키 -> 먼저 정지, 정지 확인 뒤 다시 누를 것)")
                elif now - last_send >= RESEND_S:
                    send(ser, command(motion))
                    last_send = now
            elif state == "stopping":
                still = enc is not None and now - enc_changed_at >= STILL_S
                waited = now - stop_at
                if (still and waited >= STILL_S) or waited >= STOP_TIMEOUT_S:
                    state = "ready"
                    note = "(엔코더 0.3초 정지)" if still else "(2초 경과 - 엔코더로 확인 못함, 바퀴를 눈으로 확인)"
                    motion = None
                    status("정지 확인 " + note + " -> 키를 새로 누르면 출발")
            elif state == "ready":
                fresh = pressed & set(MOTIONS)
                if fresh and len(held) == 1 and fresh == held:
                    motion = next(iter(held))
                    send(ser, command(motion))
                    last_send = now
                    run_base = list(enc) if enc is not None else None
                    last_print = now
                    state = "moving"
                    status("(출발)")

            enc_period = ENC_STOPPING_S if state == "stopping" else ENC_S
            if now - last_enc >= enc_period:     # ENC 는 데드맨을 갱신하지 않는다
                send(ser, "ENC")
                last_enc = now

            for ln in read_lines(ser):
                if ln.startswith("ENC"):
                    try:
                        new = [int(x) for x in ln.split()[1:5]]
                    except ValueError:
                        continue
                    if new != enc:
                        enc_changed_at = now
                    enc = new
                    log_only(f"ENC {enc} state={state} motion={motion} speed={speed}")
                elif ln == "READY":
                    if state == "moving":
                        state, stop_at = "stopping", now
                    say("[펌웨어] READY - 아두이노가 리셋됨 -> 정지. 원인 확인 뒤 키를 새로 누를 것")
                elif ln.startswith(("ERR", "UNK", "EVT", "WARN")) or ln == "OK CLR":
                    if ln.startswith(("EVT PWR OFF", "ERR PWR")) and state == "moving":
                        state, stop_at = "stopping", now
                    say("[펌웨어]", ln)

            if state == "moving" and enc is not None and now - last_print >= PRINT_S:
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
