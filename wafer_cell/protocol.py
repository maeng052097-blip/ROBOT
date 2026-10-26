"""셀 Mega <-> PC 시리얼 줄 프로토콜(순수 로직). 규격 문서 = PROTOCOL.md (펌웨어 작성자용).

한 줄 = '$' TYPE (',' key '=' value)* '*' HH '\\n'
  - HH = '$' 와 '*' 사이 바이트의 XOR (16 진 대문자 2 자리). NMEA 와 같은 방식.
  - ASCII 인쇄 문자만. 값에는 '$' '*' ',' '=' 공백을 쓰지 않는다(VALUE_RE).
  - 길이('$' ~ HH, 줄바꿈 제외): PC -> Mega <= 60 B (Mega 수신 버퍼 64 B), Mega -> PC <= 200 B.
  - 수신 쪽은 '\\r' 을 버리고, '$' 앞 쓰레기 바이트는 건너뛰고, 너무 긴 줄은 버린다(Decoder).
Mega 가 공정을 돌리고(권한), PC 는 표시 · 카메라 판정 · 운전 명령(원점/시작/일시정지/재개/정지/알람 해제)만 한다.
"""
import re

from wafer_cell import cell_config as C

PROTO_VERSION = C.PROTO_VERSION
MAX_PC_LINE = 60
MAX_MEGA_LINE = 200
TYPES_TO_PC = ("HEL", "ST", "SL", "EV", "CAP", "ACK")
TYPES_TO_MEGA = ("HB", "CMD", "JDG")
LIVE_CMDS = ("hello", "home", "start", "pause", "resume", "stop", "reset")
STATES = ("IDLE", "HOMING", "READY", "RUNNING", "WAIT_CART", "WAIT_NG", "PAUSED", "ALARM", "ESTOP")
PAUSE_REASONS = ("USER", "LINK", "HOLD")
CART_STATES = ("AWAY", "MOVING", "DOCKED", "LEAVING")
JUDGMENTS = ("OK", "NG", "ERR")
ACK_ERRORS = ("bad_state", "estop", "not_homed", "cell_not_clear", "unknown", "bad_arg", "busy")
SLOT_CODES = "UOKNE-"      # ① 슬롯: U 미처리 · O 꺼냄 · K 정상 원위치 · N 불량(③) · E 빈 슬롯 · - 카세트 없음
SEAT_CODES = "FWD-"        # 대차 받침: F 처리 전 · W 셀에서 처리 중 · D 끝 · - 빈 받침
TYPE_RE = re.compile(r"^[A-Z]{2,3}$")
KEY_RE = re.compile(r"^[a-z][a-z0-9]{0,3}$")
VALUE_RE = re.compile(r"^[A-Za-z0-9_.+\-/:#]*$")

# 키 종류: int · float · bool(0/1) · word(VALUE_RE) · mask5('0'/'1' x5) · slots5 · seats4
# (kind, 필수 여부)
SPEC = {
    "HEL": {"p": ("int", True), "b": ("word", True), "fw": ("word", False)},
    "ST": {"q": ("int", True), "ms": ("int", True), "st": ("word", True), "stp": ("word", True), "ai": ("int", True),
           "n": ("int", True), "k": ("int", False), "w": ("int", False),
           "th": ("float", True), "zc": ("float", True), "rx": ("float", True), "hm": ("int", True),
           "ths": ("mask5", True), "zd": ("float", True), "rd": ("float", True), "s4": ("bool", True),
           "ti": ("bool", True), "tn": ("bool", True), "d9": ("bool", True), "uv": ("bool", True),
           "au": ("bool", True), "ul": ("float", False), "pr": ("word", False), "al": ("word", True)},
    "SL": {"in": ("slots5", True), "ng": ("int", True), "cs": ("seats4", True), "ca": ("word", True),
           "cal": ("word", True), "cmv": ("word", False), "fw": ("bool", True), "fc": ("bool", True),
           "cr": ("bool", True), "uh": ("bool", True), "ins": ("int", True), "okc": ("int", True),
           "ngc": ("int", True), "emp": ("int", True), "cyc": ("float", False), "dep": ("int", True),
           "ju": ("word", False), "jw": ("int", False), "jdr": ("float", False), "jdt": ("float", False)},
    "EV": {"q": ("int", True), "lv": ("word", True), "c": ("word", True), "d": ("word", False),
           "stp": ("word", False), "n": ("int", False), "k": ("int", False), "w": ("int", False),
           "s": ("word", False)},
    "CAP": {"id": ("int", True), "n": ("int", True), "w": ("int", True)},
    "ACK": {"id": ("int", True), "ok": ("bool", True), "e": ("word", False), "st": ("word", True)},
    "HB": {"t": ("int", True)},
    "CMD": {"id": ("int", True), "c": ("word", True), "x": ("word", False)},
    "JDG": {"id": ("int", True), "r": ("word", True), "dr": ("float", True), "dt": ("float", True)},
}
ENUMS = {
    ("ST", "st"): STATES, ("ST", "pr"): PAUSE_REASONS + ("-",), ("ACK", "st"): STATES,
    ("SL", "ca"): CART_STATES, ("SL", "cal"): tuple(C.CART_SEATS) + ("-",), ("SL", "ju"): ("OK", "NG", "ERR", "-"),
    ("EV", "lv"): ("I", "W", "A"), ("EV", "s"): tuple(C.CART_SEATS) + ("-",),
    ("CMD", "c"): LIVE_CMDS, ("CMD", "x"): STATES, ("JDG", "r"): JUDGMENTS,
    ("ACK", "e"): ACK_ERRORS + ("-", "dup"),
}


class ProtoError(Exception):
    """code: no_dollar | no_star | bad_hex | bad_crc | bad_char | too_long | bad_type | bad_field"""

    def __init__(self, code, line=b""):
        super().__init__(code)
        self.code, self.line = code, line


class Msg:
    __slots__ = ("type", "fields")

    def __init__(self, typ, fields):
        self.type, self.fields = typ, fields

    def get(self, k, default=None):
        return self.fields.get(k, default)

    def __repr__(self):
        return f"Msg({self.type}, {self.fields})"


def checksum(body):
    if isinstance(body, str):
        body = body.encode("ascii")
    x = 0
    for b in body:
        x ^= b
    return x


def fmt(v):
    """값 -> 문자열. bool -> 0/1, float -> 짧게, None -> '-'."""
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        s = f"{v:.2f}".rstrip("0").rstrip(".")
        return "0" if s in ("-0", "") else s
    return str(v)


def encode(typ, fields=None, max_len=None, **kw):
    """(TYPE, {k: v}) -> b'$TYPE,k=v*HH\\n'. 순서는 dict 순서."""
    items = list((fields or {}).items()) + list(kw.items())
    if not TYPE_RE.match(typ):
        raise ProtoError("bad_type")
    parts = [typ]
    for k, v in items:
        s = fmt(v)
        if not KEY_RE.match(k) or not VALUE_RE.match(s):
            raise ProtoError("bad_field", f"{k}={s}".encode("ascii", "replace"))
        parts.append(f"{k}={s}")
    body = ",".join(parts)
    line = f"${body}*{checksum(body):02X}"
    lim = max_len or (MAX_PC_LINE if typ in TYPES_TO_MEGA else MAX_MEGA_LINE)
    if len(line) > lim:
        raise ProtoError("too_long", line.encode("ascii"))
    return (line + "\n").encode("ascii")


def parse(line, max_len=MAX_MEGA_LINE):
    """b'$TYPE,k=v*HH' (줄바꿈 있어도 됨) -> Msg. 잘못이면 ProtoError."""
    if isinstance(line, str):
        line = line.encode("ascii", "replace")
    line = line.rstrip(b"\r\n")
    if len(line) > max_len:
        raise ProtoError("too_long", line)
    if not line.startswith(b"$"):
        raise ProtoError("no_dollar", line)
    star = line.rfind(b"*")
    if star < 0:
        raise ProtoError("no_star", line)
    body, hh = line[1:star], line[star + 1:]
    if len(hh) != 2 or not all(c in b"0123456789ABCDEFabcdef" for c in hh):
        raise ProtoError("bad_hex", line)
    if any(c < 0x20 or c > 0x7E or c in b"$*" for c in body):
        raise ProtoError("bad_char", line)
    if checksum(body) != int(hh, 16):
        raise ProtoError("bad_crc", line)
    parts = body.decode("ascii").split(",")
    typ = parts[0]
    if not TYPE_RE.match(typ):
        raise ProtoError("bad_type", line)
    fields = {}
    for p in parts[1:]:
        k, eq, v = p.partition("=")
        if not eq or not KEY_RE.match(k) or not VALUE_RE.match(v) or k in fields:
            raise ProtoError("bad_field", line)
        fields[k] = v
    return Msg(typ, fields)


class Decoder:
    """바이트 조각 -> 줄 -> Msg. 쪼개진 줄 · CRLF · '$' 앞 쓰레기 · 너무 긴 줄을 견딘다."""

    def __init__(self, max_len=MAX_MEGA_LINE):
        self.buf = b""
        self.max_len = max_len
        self.errors = {}
        self.noise = 0

    def _err(self, code):
        self.errors[code] = self.errors.get(code, 0) + 1

    def feed(self, data):
        """반환: Msg 목록(잘못된 줄은 errors 에 세고 버림)."""
        out = []
        self.buf += data
        while True:
            nl = self.buf.find(b"\n")
            if nl < 0:
                if len(self.buf) > self.max_len + 4:
                    self._err("too_long")
                    cut = self.buf.find(b"$", 1)
                    self.buf = self.buf[cut:] if cut > 0 else b""
                    continue
                break
            raw, self.buf = self.buf[:nl], self.buf[nl + 1:]
            d = raw.rfind(b"$")                    # '$' 가 여럿이면 앞은 끊긴 줄 -> 마지막부터
            if d < 0:
                if raw.strip(b"\r \x00"):
                    self.noise += 1
                continue
            if d > 0:
                self.noise += 1
                raw = raw[d:]
            try:
                out.append(parse(raw, self.max_len))
            except ProtoError as e:
                self._err(e.code)
        return out


def _ok_kind(kind, v):
    if v == "-" and kind in ("float", "int", "word"):
        return True
    try:
        if kind == "int":
            int(v)
        elif kind == "float":
            float(v)
        elif kind == "bool":
            return v in ("0", "1")
        elif kind == "mask5":
            return len(v) == 5 and set(v) <= set("01")
        elif kind == "slots5":
            return len(v) == C.SLOT_COUNT and set(v) <= set(SLOT_CODES)
        elif kind == "seats4":
            return len(v) == len(C.CART_SEATS) and set(v) <= set(SEAT_CODES)
        return True
    except ValueError:
        return False


def validate(msg):
    """SPEC 대조. 문제 목록(빈 목록 = 통과). 모르는 키는 앞으로의 확장을 위해 허용(경고 없음)."""
    spec = SPEC.get(msg.type)
    if spec is None:
        return [f"unknown type {msg.type}"]
    probs = []
    for k, (kind, req) in spec.items():
        if k not in msg.fields:
            if req:
                probs.append(f"missing {k}")
            continue
        v = msg.fields[k]
        if not _ok_kind(kind, v):
            probs.append(f"bad {k}={v}")
        enum = ENUMS.get((msg.type, k))
        if enum and v not in enum:
            probs.append(f"enum {k}={v}")
    if msg.type in ("ST", "EV") and msg.fields.get("stp") not in (None, "-"):
        from wafer_cell import sequence as Q
        if msg.fields["stp"] not in Q.step_keys():
            probs.append(f"step {msg.fields['stp']}")
    return probs


def typed(msg):
    """문자열 값 -> 파이썬 값(SPEC 기준). '-' -> None."""
    spec = SPEC.get(msg.type, {})
    out = {}
    for k, v in msg.fields.items():
        kind = spec.get(k, ("word", False))[0]
        if v == "-" and kind in ("int", "float", "word"):
            out[k] = None
        elif kind == "int":
            out[k] = int(v)
        elif kind == "float":
            out[k] = float(v)
        elif kind == "bool":
            out[k] = v == "1"
        elif kind == "mask5":
            out[k] = [c == "1" for c in v]
        else:
            out[k] = v
    return out


def mask5(bits):
    return "".join("1" if b else "0" for b in bits)


def homed_bits(homed):
    return (1 if homed.get("theta") else 0) | (2 if homed.get("z") else 0) | (4 if homed.get("r") else 0)


def homed_from_bits(v):
    v = int(v or 0)
    return {"theta": bool(v & 1), "z": bool(v & 2), "r": bool(v & 4)}
