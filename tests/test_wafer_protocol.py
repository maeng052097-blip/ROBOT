"""tests/test_wafer_protocol.py — 셀 Mega <-> PC 줄 프로토콜 단위테스트(하드웨어 · pyserial 불필요).

  - 체크섬(XOR) · encode/parse 왕복 · SPEC 검증(validate) · typed 변환
  - 오류 코드: no_dollar / no_star / bad_hex / bad_crc / bad_char / too_long / bad_type / bad_field
  - Decoder: 바이트 하나씩 쪼개진 줄, CRLF, '$' 앞 쓰레기, 한 조각에 두 줄, 너무 긴 줄 뒤 복구
  - 길이 한계: PC -> Mega 최악 줄 <= 60 B (Mega 수신 버퍼 64), Mega -> PC 최악 ST/SL/EV <= 200 B
  - PROTOCOL.md 에 모든 메시지 종류와 키가 적혀 있는지(펌웨어 작성자 문서와 코드가 어긋나지 않게)

실행: py -3.13 tests/test_wafer_protocol.py
"""
import re
import sys
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def main():
    from wafer_cell import protocol as P
    from wafer_cell import sequence as Q

    print("test_wafer_protocol:")
    x = 0
    for b in b"HB,t=7":
        x ^= b
    assert P.checksum("HB,t=7") == x
    line = P.encode("HB", {"t": 7})
    assert line == f"$HB,t=7*{x:02X}\n".encode(), line
    m = P.parse(line)
    assert m.type == "HB" and m.fields == {"t": "7"} and not P.validate(m)
    assert P.parse(line.replace(b"\n", b"\r\n")).fields == {"t": "7"}
    assert P.parse(line.decode().lower().replace("hb", "HB").encode()).type == "HB"   # 소문자 16진도 받음
    print("  OK checksum (XOR between $ and *) / encode / parse / CRLF")

    samples = {
        "HEL": {"p": 1, "b": "A1B2C3", "fw": "r22-0.1"},
        "ST": {"q": 65535, "ms": 4294967295, "st": "WAIT_CART", "stp": "U7", "ai": 10, "n": 5, "k": 5, "w": 999,
               "th": -44.99, "zc": 245.0, "rx": 85.5, "hm": 7, "ths": "11111", "zd": 244.1, "rd": 189.5,
               "s4": True, "ti": True, "tn": True, "d9": True, "uv": True, "au": True, "ul": 10.0, "pr": "LINK",
               "al": "R_NOT_RETRACTED"},
        "SL": {"in": "KKNOU", "ng": 5, "cs": "DWF-", "ca": "LEAVING", "cal": "RR", "cmv": "LATERAL:-200",
               "fw": True, "fc": False, "cr": True, "uh": True, "ins": 9999, "okc": 9999, "ngc": 9999,
               "emp": 9999, "cyc": 128.25, "dep": 999, "ju": "NG", "jw": 999, "jdr": -3.5, "jdt": -0.5},
        "EV": {"q": 65535, "lv": "A", "c": "UV_INTERLOCK", "d": "TH60+ANGLE+RX+ZC+HAND+JUDGE+D9", "stp": "U6",
               "n": 5, "k": 5, "w": 999, "s": "RR"},
        "CAP": {"id": 65535, "n": 5, "w": 999},
        "ACK": {"id": 65535, "ok": False, "e": "cell_not_clear", "st": "WAIT_CART"},
        "HB": {"t": 65535},
        "CMD": {"id": 65535, "c": "resume", "x": "WAIT_CART"},
        "JDG": {"id": 65535, "r": "ERR", "dr": -3.5, "dt": -0.5},
    }
    assert set(samples) == set(P.SPEC)
    for typ, f in samples.items():
        ln = P.encode(typ, f)
        lim = P.MAX_PC_LINE if typ in P.TYPES_TO_MEGA else P.MAX_MEGA_LINE
        assert len(ln.rstrip(b"\n")) <= lim, (typ, len(ln))
        mm = P.parse(ln, max_len=lim)
        assert mm.type == typ and not P.validate(mm), (typ, P.validate(mm))
    ln = P.encode("EV", dict(samples["EV"], d="X" * 40))
    assert len(ln) <= P.MAX_MEGA_LINE
    t = P.typed(P.parse(P.encode("ST", samples["ST"])))
    assert t["s4"] is True and t["ths"] == [True] * 5 and t["th"] == -44.99 and t["q"] == 65535
    t = P.typed(P.parse(P.encode("ST", dict(samples["ST"], zd=None, al=None))))
    assert t["zd"] is None and t["al"] is None
    print(f"  OK every type round-trips + validates; worst-case PC->Mega <= {P.MAX_PC_LINE} B, Mega->PC <= {P.MAX_MEGA_LINE} B")

    bad = {
        b"HB,t=1*00": "no_dollar", b"$HB,t=1": "no_star", b"$HB,t=1*G0": "bad_hex", b"$HB,t=1*0": "bad_hex",
        b"$HB,t=1*00": "bad_crc", b"$HB,t=\x01*" + b"%02X" % P.checksum(b"HB,t=\x01"): "bad_char",
        b"$" + b"A" * 300 + b"*00": "too_long",
    }
    for raw, code in bad.items():
        try:
            P.parse(raw)
            raise AssertionError(f"no error for {raw[:20]}")
        except P.ProtoError as e:
            assert e.code == code, (raw[:20], e.code, code)
    for body in ("hb,t=1", "HB,t", "HB,T=1", "HB,t=a b", "HB,t=1,t=2"):
        raw = f"${body}*{P.checksum(body):02X}".encode()
        try:
            P.parse(raw)
            raise AssertionError(body)
        except P.ProtoError as e:
            assert e.code in ("bad_type", "bad_field", "bad_char"), (body, e.code)
    for bad_kw in ({"t": "a,b"}, {"t": "x=y"}, {"T": 1}, {"t": "a*"}):
        try:
            P.encode("HB", bad_kw)
            raise AssertionError(bad_kw)
        except P.ProtoError:
            pass
    try:
        P.encode("CMD", {"id": 1, "c": "x" * 60})
        raise AssertionError("long PC line accepted")
    except P.ProtoError as e:
        assert e.code == "too_long"
    assert P.validate(P.parse(P.encode("ST", dict(samples["ST"], st="FLYING"))))
    assert P.validate(P.parse(P.encode("ST", dict(samples["ST"], stp="Z9"))))
    assert P.validate(P.parse(P.encode("CMD", {"id": 1, "c": "fault"})))
    assert P.validate(P.parse(P.encode("SL", dict(samples["SL"], **{"in": "KKX"}))))
    assert "missing t" in P.validate(P.parse(P.encode("HB", {})))
    print("  OK error codes + SPEC rejects bad enum / unknown step / non-LIVE command / bad slot codes / missing key")

    d = P.Decoder()
    stream = b"\x00\xffnoise\n" + P.encode("HB", {"t": 1}).replace(b"\n", b"\r\n") + b"junk" + P.encode("HB", {"t": 2})
    got = []
    for i in range(len(stream)):
        got += d.feed(stream[i:i + 1])
    assert [m.fields["t"] for m in got] == ["1", "2"], got
    assert d.noise >= 2
    two = P.encode("HB", {"t": 3}) + P.encode("HB", {"t": 4})
    assert [m.fields["t"] for m in d.feed(two)] == ["3", "4"]
    d.feed(b"$" + b"A" * 500)
    assert d.errors.get("too_long", 0) >= 1
    assert [m.fields["t"] for m in d.feed(b"\n" + P.encode("HB", {"t": 5}))] == ["5"]
    cut = P.encode("HB", {"t": 6})
    assert [m.fields["t"] for m in d.feed(cut[:5] + P.encode("HB", {"t": 7}))] == ["7"]   # 끊긴 줄 + 새 줄
    print("  OK Decoder: byte-by-byte, CRLF, garbage before $, two lines/chunk, overlong recovery, cut line")

    doc = (ROOT / "wafer_cell" / "PROTOCOL.md").read_text(encoding="utf-8")
    for typ, keys in P.SPEC.items():
        assert f"`{typ}`" in doc, typ
        for k in keys:
            assert re.search(rf"`{k}`", doc), (typ, k)
    for code in P.ACK_ERRORS + P.LIVE_CMDS + P.STATES:
        assert code in doc, code
    for k in Q.step_keys():
        assert re.search(rf"\b{k}\b", doc), k
    assert "60" in doc and "200" in doc and "XOR" in doc
    print("  OK PROTOCOL.md lists every type, key, command, state, ACK error and step key")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
