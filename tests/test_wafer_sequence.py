"""tests/test_wafer_sequence.py — CELL_V9 시퀀스/인터록 단위테스트(하드웨어 불필요).

  - 원점 절차 순서: R 수축 -> Z 최하단 -> ZC105 -> θ1 -> 최하단 (θ1 은 반드시 Z 뒤, ZC105 에서만)
  - 장당 21단계 수치(Sn-66, +5, 크래들 192/181/187)와 보강(단계 10 S3, 단계 12 Z->R, 단계 21 역회전)
  - 모든 선회는 ZC105 · RX0 · 직전 S3 확인 상태에서만

실행: py -3.13 tests/test_wafer_sequence.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def walk(steps, zc, rx):
    """동작을 순서대로 따라가며 (단계키, 동작, 그 시점 zc, rx, 직전 S3 확인 여부) 를 낸다."""
    s3_ok = rx == 0
    for st in steps:
        for a in st.actions:
            yield st, a, zc, rx, s3_ok
            if a.kind == "z":
                zc = a.value
            elif a.kind == "r":
                rx = a.value
                s3_ok = False
            elif a.kind == "expect" and a.sensor == "S3" and a.expect:
                s3_ok = rx == 0


def main():
    from wafer_cell import cell_config as C
    from wafer_cell import sequence as Q

    print("test_wafer_sequence:")
    h = Q.homing_steps()
    kinds = [(a.kind, a.value) for st in h for a in st.actions if a.kind in ("z", "r", "rot")]
    assert kinds == [("r", 0.0), ("z", C.ZC_HOME), ("z", C.ZC_SWING), ("rot", 0.0), ("z", C.ZC_HOME)], kinds
    print("  OK homing order R -> Z bottom -> ZC105 -> theta1 -> bottom")

    for n in (1, 5, 10):
        s = Q.wafer_steps(n)
        assert [st.key for st in s] == [str(i) for i in range(1, 22)]
        sn = C.slot_z(n)
        assert s[0].actions[0].value == sn - 66.0
        assert s[2].actions[0].value == sn - 61.0
        assert s[2].actions[1].kind == "check_pick"
    s = Q.wafer_steps(3)
    by = {st.key: st for st in s}
    assert [a.value for a in by["7"].actions] == [192.0]
    assert by["9"].actions[0].value == 181.0 and by["9"].actions[1].expect is False
    assert by["13"].actions[0].value == 187.0
    assert any(a.kind == "expect" and a.sensor == "S3" for a in by["10"].actions), "step 10 needs S3"
    assert [a.kind for a in by["12"].actions] == ["z", "r"], "step 12 = Z first, then R"
    rot21 = [a for a in by["21"].actions if a.kind == "rot"][0]
    assert rot21.value == 0.0 and rot21.via == (90.0,)
    assert by["17"].actions[0].kind == "wait_s5"
    print("  OK 21 steps: numbers + added S3 at 10, Z->R at 12, 180->90->0 at 21")

    rots = 0
    for steps, zc0 in ((h, 150.0), (Q.wafer_steps(1), C.ZC_SWING), (Q.wafer_steps(10), C.ZC_SWING)):
        for st, a, zc, rx, s3_ok in walk(steps, zc0, 0.0):
            if a.kind == "rot":
                rots += 1
                assert zc == C.ZC_SWING, (st.key, zc)
                assert rx == 0.0 and s3_ok, (st.key, rx, s3_ok)
                assert Q.can_rotate(zc, rx, True, carrying=True)[0]
    assert rots == 7
    print("  OK every rotation at ZC105, RX0, S3 confirmed (7 rotations)")

    ok, why = Q.can_rotate(95.0, 0.0, True)
    assert not ok and "ZC" in why
    assert not Q.can_rotate(105.0, 51.0, True)[0]
    assert not Q.can_rotate(105.0, 0.0, False)[0]
    assert Q.can_rotate(105.0, 0.0, True, carrying=True)[0]
    print("  OK can_rotate rejects ZC95 / RX51 / no S3")

    e = Q.empty_slot_steps(4)
    assert [st.key for st in e] == ["4", "5"]
    cat = Q.step_catalog()
    assert len(cat) == 5 + 21 + 1 and cat[-1]["key"] == "HO"
    print("  OK empty-slot branch and catalog")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
