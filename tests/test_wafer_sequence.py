"""tests/test_wafer_sequence.py — R22 공정 시퀀스 단위테스트(순수 로직, 하드웨어 불필요).

  - 단계 키 39 개가 서로 다르고 묶음 이름이 있음, describe(키, 슬롯) 문구, action_at
  - 자세 따라가기(walk): 원점 -> 대차->① -> 웨이퍼(정상 · 불량) -> ①->대차 를 지령값으로 밟으며
      모든 선회가 그 자리 인터록(can_rotate)을 통과, 옆길은 θ180~220 · 카세트 운반은 180~270 · Z241 에서만,
      UV 탑 앞에서 R 전진은 ZC 113.5~121.5 에서만, ZC 는 245 이하, UV 조사는 ZC <= 110 · R 후진에서
  - S4 '없음' 확인은 언제나 R 후진 확인 바로 뒤(놓은 직후에는 아직 보임)
  - ① 앞에서 웨이퍼를 든 채 R 후진 상태로 Z 를 바꾸는 동작이 없음(옆길에서만, 슬롯 안 들기/놓기는 제외)
  - UV 켜기 7 조건 진리표, 보정 창(δr -3.5~+0.5 · |δθ| <= 0.5) 경계

실행: py -3.13 tests/test_wafer_sequence.py
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def main():
    from wafer_cell import cell_config as C
    from wafer_cell import sequence as Q

    print("test_wafer_sequence:")
    keys = Q.step_keys()
    assert len(keys) == len(set(keys)) == 39, keys
    cat = Q.step_catalog()
    assert [c["key"] for c in cat] == keys and all(c["group"] in Q.GROUP_LABELS for c in cat)
    assert all(Q.STAGE_OF_GROUP[g] in (None,) + tuple(k for k, _ in Q.STAGES) for g in Q.GROUP_LABELS)
    assert "슬롯 n" in Q.describe("P2")[1] and "슬롯 3" in Q.describe("P2", 3)[1]
    assert "슬롯 k" in Q.describe("N2")[1] and "슬롯 4" in Q.describe("N2", None, 4)[1]
    assert "앞 오른" in Q.describe("C1", seat="FR")[1] and "(" not in Q.describe("C1")[1]
    assert Q.action_at("U6", 1).kind == "uv" and Q.action_at("P3", 1).kind == "check_pick"
    assert Q.action_at("P2", 0, 4).value == C.pick_z(4) and Q.action_at("ZZ", 0) is None
    assert Q.action_count("I4") == len(Q.inspect_steps(1)[3].actions)
    print("  OK 39 unique step keys, groups, describe(slot/NG slot/seat), action_at")

    # ---- 지령값 따라가기
    pose = {"th": 150.0, "z": 140.0, "r": 30.0}
    carry = {"wafer": False, "cass": False}
    rots, problems = [], []

    def walk(steps, wafer=None, cass=None):
        for st in steps:
            acts = st.actions
            for i, a in enumerate(acts):
                if a.kind == "z":
                    if abs(C.wrap_deg(pose["th"] - C.STATIONS["input"])) < 1 and carry["wafer"] \
                            and pose["r"] <= C.RX_RET_OK and abs(a.value - pose["z"]) > 0.01:
                        problems.append(f"{st.key}: Z move at input while carrying a wafer")
                    if abs(C.wrap_deg(pose["th"] - C.STATIONS["uv"])) < 1 and pose["r"] > C.RX_RET_OK \
                            and not C.UV_ZC_HAND <= a.value <= C.UV_ZC_IN:
                        problems.append(f"{st.key}: Z {a.value} under UV tower")
                    if a.value > C.ZC_SOFT_MAX:
                        problems.append(f"{st.key}: Z {a.value} > soft max")
                    pose["z"] = a.value
                elif a.kind == "r":
                    if abs(C.wrap_deg(pose["th"] - C.STATIONS["uv"])) < 1 and a.value > C.RX_RET_OK \
                            and not C.UV_ZC_HAND <= pose["z"] <= C.UV_ZC_IN:
                        problems.append(f"{st.key}: R out under UV at Z {pose['z']}")
                    pose["r"] = a.value
                elif a.kind == "rot":
                    ok, why = Q.can_rotate(pose["z"], pose["r"] <= C.RX_RET_OK, pose["th"], a.value, a.mode,
                                           carry["wafer"], carry["cass"])
                    if not ok:
                        problems.append(f"{st.key}: rotate {pose['th']}->{a.value} {a.mode} blocked {why}")
                    rots.append((st.key, pose["th"], a.value, a.mode, pose["z"]))
                    pose["th"] = a.value
                elif a.kind == "home_th":
                    pose["th"] = C.STATIONS["inspect"]
                elif a.kind == "uv":
                    if pose["z"] > C.UV_ZC_LAMP_MAX or pose["r"] > C.RX_RET_OK or pose["th"] != C.STATIONS["uv"]:
                        problems.append(f"{st.key}: UV at bad pose {pose}")
                elif a.kind == "expect" and a.sensor == "S4" and not a.expect:
                    if i == 0 or acts[i - 1].kind != "ret":
                        problems.append(f"{st.key}: S4 none not right after R retract check")
            if wafer is not None and st.key == wafer[0]:
                carry["wafer"] = wafer[1]
            if cass is not None and st.key == cass[0]:
                carry["cass"] = cass[1]

    walk(Q.homing_steps())
    walk(Q.cart_in_steps("FL")[:3])
    carry["cass"] = True                                       # C3 들기 뒤 ~ C5 놓기 전 (C4 = 카세트 든 선회)
    walk(Q.cart_in_steps("FL")[3:4])
    carry["cass"] = False
    walk(Q.cart_in_steps("FL")[4:])
    for n, ng in ((1, False), (2, True), (5, False)):
        walk(Q.input_steps(n)[:2])
        carry["wafer"] = True                                  # P3 들기 뒤
        walk(Q.input_steps(n)[2:] + Q.inspect_steps(n)[:2], wafer=("I2", False))
        walk(Q.inspect_steps(n)[2:], wafer=("I4", True))
        if ng:
            walk(Q.ng_steps(C.SLOT_COUNT), wafer=("N2", False))
        else:
            walk(Q.uv_steps()[:4], wafer=("U4", False))
            walk(Q.uv_steps()[4:], wafer=("U7", True))
            walk(Q.return_steps(n), wafer=("R4", False))
    walk(Q.empty_steps(3))
    walk(Q.input_steps(1)[:1])
    walk(Q.cart_out_steps("FL")[:3])
    carry["cass"] = True
    walk(Q.cart_out_steps("FL")[3:5], cass=("L5", False))
    walk(Q.cart_out_steps("FL")[5:] + Q.standby_steps())
    assert not problems, problems
    for key, a, b, mode, z in rots:
        lo, hi = min(a, b), max(a, b)
        if mode == "side":
            s_lo, s_hi = sorted((C.STATIONS["input"], C.SIDE_THETA))
            assert lo >= s_lo - 1e-6 and hi <= s_hi + 1e-6, (key, a, b)
        if mode == "carry":
            assert lo >= 180 - 1e-6 and hi <= 270 + 1e-6 and z == C.CASS_Z_CARRY, (key, a, b, z)
        if mode == "swing":
            assert z == C.ZC_SWING, (key, z)
    assert any(r[3] == "carry" and r[2] == 180 for r in rots) and any(r[3] == "carry" and r[2] == 270 for r in rots)
    assert sum(1 for r in rots if r[3] == "side") >= 4
    # 사용자 요청: ① 에서 꺼내 바로 검사 쪽으로(반대 방향 없음), 되돌릴 때도 한 방향
    i1 = [(a, b) for k, a, b, m, z in rots if k == "I1"]
    assert i1 and all(b < a for a, b in i1) and i1[0][0] == C.STATIONS["input"] and i1[-1][1] == C.STATIONS["inspect"], i1
    ret = [(a, b) for k, a, b, m, z in rots if k in ("R1", "R3")]
    assert ret and all(b > a for a, b in ret) and ret[-1][1] == C.STATIONS["input"], ret
    assert C.STATIONS["inspect"] < C.SIDE_THETA < C.STATIONS["input"]
    print(f"  OK walk {len(rots)} rotations: interlock passes, side {C.SIDE_THETA:g}-180, carry 180-270 @ Z241, swing @ Z105")
    print("  OK one direction: pick 180 -> side -> inspect (I1 decreasing), return uv -> side -> 180 (R1/R3 increasing)")
    print("  OK no Z move at input while carrying, UV zone respected, UV lamp at ZC<=110 & R retracted, S4-none after retract")

    assert Q.can_rotate(205, True, 180, C.SIDE_THETA, "side")[0] and Q.can_rotate(205, True, C.SIDE_THETA, 180, "side")[0]
    assert not Q.can_rotate(205, False, 180, C.SIDE_THETA, "side")[0]
    assert "SIDE_PATH" in Q.can_rotate(205, True, 180, 220, "side")[1]          # 예전 옆길(문서 220)은 이제 금지
    assert "SIDE_PATH" in Q.can_rotate(205, True, 120, 180, "side")[1]
    assert "Z_NOT_SWING" in Q.can_rotate(121.5, True, 0, 120, "swing")[1]
    assert "CASS_SWING" in Q.can_rotate(105, True, 180, 270, "swing", carrying_cassette=True)[1]
    assert "CARRY_PATH" in Q.can_rotate(241, True, 180, 0, "carry")[1]
    assert "THETA_RANGE" in Q.can_rotate(105, True, 0, -60, "swing")[1]
    assert Q.can_rotate(183, True, 120, 120.5, "fine")[0] and not Q.can_rotate(183, True, 120, 120.6, "fine")[0]
    assert not Q.can_rotate(192, True, 120, 120.2, "fine")[0]
    print("  OK can_rotate: side/carry/fine/swing rules + R retract required")

    uv = C.STATIONS["uv"]                       # UV 자리 θ60 (사용자 변경) -> 자리 센서 TH60
    assert C.UV_TH_SENSOR == "TH60" and C.TH_OF["ng"] == "TH0"
    base = dict(th_uv_low=True, theta=uv, rx_ret_ok=True, zc=105.0, handed_over=True, judgment_ok=True, d9_ok=True)
    assert Q.uv_interlock(**base) == []
    flips = {"th_uv_low": (False, C.UV_TH_SENSOR), "theta": (uv + 0.21, "ANGLE"), "rx_ret_ok": (False, "RX"),
             "zc": (110.01, "ZC"), "handed_over": (False, "HAND"), "judgment_ok": (False, "JUDGE"), "d9_ok": (False, "D9")}
    for k, (v, code) in flips.items():
        assert Q.uv_interlock(**dict(base, **{k: v})) == [code], (k, Q.uv_interlock(**dict(base, **{k: v})))
    assert Q.uv_interlock(**dict(base, theta=uv - 0.2, zc=110.0)) == []
    assert "ANGLE" in Q.uv_interlock(**dict(base, theta=0.0))      # ③ 자리(θ0)에서는 켜지 않음
    assert len(Q.uv_interlock(False, 5.0, False, 120.0, False, False, False)) == 7
    print("  OK UV interlock truth table (7 conditions, each alone)")

    for dr, dth, want in ((-3.5, 0, True), (-3.51, 0, False), (0.5, 0, True), (0.51, 0, False),
                          (0, 0.5, True), (0, -0.5, True), (0, -0.51, False), (0, 0, True)):
        assert Q.corr_ok(dr, dth) is want, (dr, dth)
    print("  OK correction window boundaries")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
