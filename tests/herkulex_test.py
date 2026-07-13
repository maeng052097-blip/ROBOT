"""herkulex_test.py -- Herkulex DRS-0601 서보 1개 기본 제어(스캔/토크/이동/위치읽기).

하드웨어: CP2102(USB-TTL)로 PC <-> 서보 직결. 서보 전원 = 별도 12V. (아두이노 불필요)
의존: pyserial (py -3.13 에 이미 있음). 라이브러리 설치 없이 프로토콜을 직접 구현.

실행: py -3.13 tests/herkulex_test.py --port COM8
      (라이다도 CP210x 라 COM8 을 공유 -> 이 테스트 동안은 라이다 빼두기)

콘솔 명령(치고 Enter):
  scan          : 버스에서 서보 ID 찾기(0~253 훑음, 몇 초 걸림)
  id <n>        : 대상 서보 ID 지정(기본 253 = 공장기본)
  p             : 현재 위치 읽기(토크 없어도 됨)
  on / off      : 토크 켜기 / 끄기
  g <0-1023>    : 그 위치로 이동(토크 ON 상태에서). 512=중앙 근처
  q             : 종료(토크 끄고 닫음)

★안전: 팔을 자유롭게(경로에 사람/물체 없이). 먼저 'p' 로 현재 위치를 보고, 'on' 한 뒤
        현재값에서 조금씩(예 +-50) 움직여 방향/범위를 익히세요. 큰 점프는 빠르게 휩니다.
※ 이동값 0~1023 은 표준 범위입니다. DRS-0601 실제 각도 범위는 Manager/실측으로 확인하세요.
"""
import argparse
import sys
import time

try:
    sys.stdout.reconfigure(errors="replace")
    sys.stderr.reconfigure(errors="replace")
except Exception:
    pass

import serial

# Herkulex 명령
RAM_WRITE = 0x03
RAM_READ = 0x04
S_JOG = 0x06
STAT = 0x07

# RAM 레지스터
REG_STATUS = 48      # 2바이트: status error / status detail (0 쓰면 클리어)
REG_TORQUE = 52      # 1바이트: 0x00=free, 0x40=break, 0x60=torque on
REG_ABS_POS = 60     # 2바이트: 현재 절대위치

ERR_BITS = ["입력전압 한계초과", "POT(각도)한계 초과", "온도 한계초과", "잘못된 패킷",
            "과부하(Overload)", "드라이버 이상", "EEP 손상", "(reserved)"]
DET_BITS = ["이동중", "제위치(InPosition)", "체크섬 오류", "알수없는 명령",
            "REG범위 초과", "가비지 수신", "MOTOR_ON", "(reserved)"]


def _checksum(psize, pid, cmd, data):
    x = psize ^ pid ^ cmd
    for d in data:
        x ^= d
    c1 = x & 0xFE
    c2 = (~c1) & 0xFE
    return c1, c2


def send(ser, pid, cmd, data=()):
    data = bytes(data)
    psize = 7 + len(data)                     # 헤더7 + 데이터
    c1, c2 = _checksum(psize, pid, cmd, data)
    pkt = bytes([0xFF, 0xFF, psize, pid, cmd, c1, c2]) + data
    ser.reset_input_buffer()
    ser.write(pkt)


def recv(ser, timeout=0.15):
    ser.timeout = timeout
    data = ser.read(40)                       # 한 응답은 보통 <20바이트
    i = data.find(b"\xFF\xFF")
    if i < 0 or len(data) < i + 3:
        return None
    psize = data[i + 2]
    if len(data) < i + psize:
        return None
    return data[i:i + psize]


def torque(ser, pid, on=True):
    send(ser, pid, RAM_WRITE, [REG_TORQUE, 1, 0x60 if on else 0x00])


def move(ser, pid, pos, playtime=60, led=0x04):
    pos = max(0, min(1023, int(pos)))         # 0~1023
    # S_JOG data: [playtime, posLSB, posMSB, SET(0x04=위치제어+초록LED), ID]
    send(ser, pid, S_JOG, [playtime & 0xFF, pos & 0xFF, (pos >> 8) & 0xFF, led, pid])


def read_pos(ser, pid):
    send(ser, pid, RAM_READ, [REG_ABS_POS, 2])
    r = recv(ser)
    # 응답: [FF FF psize pid cmd cs1 cs2 addr len d0 d1 err detail]
    if r and len(r) >= 11:
        return r[9] | (r[10] << 8)
    return None


def ping(ser, pid):
    send(ser, pid, STAT)
    return recv(ser, 0.03) is not None


def read_status(ser, pid):
    """STAT -> (error, detail) 바이트. 응답 없으면 None."""
    send(ser, pid, STAT)
    r = recv(ser)
    # STAT ACK: [FF FF psize pid cmd cs1 cs2 err detail]
    if r and len(r) >= 9:
        return r[7], r[8]
    return None


def print_status(ser, pid):
    st = read_status(ser, pid)
    if st is None:
        print("  응답 없음(통신 확인)")
        return
    err, det = st
    print(f"  status error=0x{err:02X} detail=0x{det:02X}")
    if err == 0:
        print("    오류 없음")
    else:
        for b in range(8):
            if err & (1 << b):
                print(f"    [오류] {ERR_BITS[b]}")
    for b in range(8):
        if det & (1 << b):
            print(f"    (상태) {DET_BITS[b]}")


def clear_status(ser, pid):
    send(ser, pid, RAM_WRITE, [REG_STATUS, 2, 0, 0])   # error/detail = 0


def scan(ser):
    found = []
    for pid in range(0, 254):
        if ping(ser, pid):
            found.append(pid)
    return found


def loopback(ser):
    """CP2102 자가진단: TXD와 RXD를 점퍼로 직접 연결한 상태에서 실행.
    보낸 바이트가 그대로 돌아오면 어댑터/포트/드라이버는 정상 -> 문제는 서보쪽 배선."""
    ser.reset_input_buffer()
    probe = b"\xA5\x5A\x55\xAA"
    ser.write(probe)
    time.sleep(0.1)
    back = ser.read(10)
    print(f"  보냄 {probe.hex()} -> 받음 {back.hex() if back else '(없음)'}")
    if back == probe:
        print("  [OK] 루프백 성공: CP2102/포트 정상. 문제는 서보쪽(GND공통/TXRX교차/레벨/핀방향)")
    elif back:
        print("  [부분] 뭔가 오지만 다름 -> 점퍼 접촉/보드레이트 확인")
    else:
        print("  [실패] 에코 없음 -> TXD-RXD 점퍼가 제대로 물렸는지, 이 COM이 CP2102가 맞는지 확인")


def main():
    ap = argparse.ArgumentParser(description="Herkulex DRS-0601 1개 기본 제어")
    ap.add_argument("--port", default="COM8", help="CP2102 COM 포트(기본 COM8)")
    ap.add_argument("--id", type=int, default=253, help="서보 ID(공장기본 253)")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, 115200, timeout=0.1)
    except Exception as e:
        print(f"[실패] 시리얼 열기 {args.port}: {e}")
        print("  - CP2102 드라이버/COM 번호 확인, 다른 프로그램(라이다 등)이 포트 점유 중인지 확인")
        return
    time.sleep(0.3)
    ser.reset_input_buffer()
    pid = args.id
    print(f"[연결] {args.port} @115200  대상 ID={pid}")
    print("명령: scan | id <n> | p | on | off | g <0-1023> | q")

    try:
        while True:
            try:
                parts = input("> ").strip().split()
            except (EOFError, KeyboardInterrupt):
                break
            if not parts:
                continue
            c = parts[0].lower()
            if c == "q":
                break
            elif c == "scan":
                print("  스캔 중(몇 초)...")
                f = scan(ser)
                if f:
                    pid = f[0]
                    print("  찾은 ID:", f, "-> 대상 ID =", pid)
                else:
                    print("  응답 없음. 배선(TX<->RX 교차)/GND공통/12V/로직레벨(3.3V) 확인")
            elif c == "id" and len(parts) >= 2:
                pid = int(parts[1]); print("  대상 ID =", pid)
            elif c == "p":
                print("  위치:", read_pos(ser, pid))
            elif c == "on":
                torque(ser, pid, True); print("  토크 ON (id", pid, ")")
            elif c == "off":
                torque(ser, pid, False); print("  토크 OFF")
            elif c == "g" and len(parts) >= 2:
                move(ser, pid, int(parts[1])); print("  이동 ->", parts[1])
            elif c == "loop":
                loopback(ser)
            elif c == "stat":
                print_status(ser, pid)
            elif c == "clear":
                clear_status(ser, pid); print("  오류 클리어 전송"); print_status(ser, pid)
            else:
                print("  ? (scan/id/p/on/off/g <pos>/stat/clear/loop/q)")
    finally:
        try:
            torque(ser, pid, False)
            ser.close()
        except Exception:
            pass
        print("종료(토크 OFF)")


if __name__ == "__main__":
    main()
