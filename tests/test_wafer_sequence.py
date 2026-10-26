"""tests/test_wafer_sequence.py — R22 공정 시퀀스 단위테스트(순수 로직, 하드웨어 불필요).

  - 단계 키 39 개가 서로 다르고 묶음 이름이 있음, describe(키, 슬롯) 문구, action_at
  - 자세 따라가기(walk): 원점 -> 대차->① -> 웨이퍼(정상 · 불량) -> ①->대차 를 지령값으로 밟으며
      모든 선회가 그 자리 인터록(can_rotate)을 통과, 높이 선회는 ② <-> ① · ② 크래들 띠 밖에서만, 카세트 운반은 180~270 · Z241 에서만,
      UV 탑 앞에서 R 전진은 ZC 113.5~121.5 에서만, ZC 는 245 이하, UV 조사는 ZC <= 110 · R 후진에서
  - S4 '없음' 확인은 언제나 R 후진 확인 바로 뒤(놓은 직후에는 아직 보임)
  - ① 앞에서 웨이퍼를 든 채 R 후진 상태로 Z 를 바꿀 때 다른 슬롯 웨이퍼 높이를 지나지 않음(슬롯 4 의 transfer_z 만)
  - 사용자 요청: ① 에서 뺀 뒤 Z105 로 내리지 않고 그대로 ② 로(I1 회전 1 번 180 -> 120), UV 뒤에도 ② 앞에서 높이만 맞추고 ① 로
  - UV 켜기 7 조건 진리표, 보정 창(δr -3.5~+0.5 · |δθ| <= 0.5) 경계
  - 동작 과정 그림(flow_diagram): 웨이퍼 1 장 시간 = 시뮬 실측(±0.2 s), 단계 키를 모두 담음, 커밋된 docs/process_flow.svg 가 최신

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
    cur = {"n": None}
    carry = {"wafer": False, "cass": False}
    rots, problems = [], []

    def walk(steps, wafer=None, cass=None):
        for st in steps:
            acts = st.actions
            for i, a in enumerate(acts):
                if a.kind == "z":
                    if abs(C.wrap_deg(pose["th"] - C.STATIONS["input"])) < 1 and carry["wafer"] \
                            and pose["r"] <= C.RX_RET_OK and abs(a.value - pose["z"]) > 0.01:
                        # 다른 슬롯(모두 찼다고 봄) 웨이퍼 높이를 지나면 충돌 - 후진 운반 웨이퍼가 ① 웨이퍼와 27 mm 겹침
                        b0 = min(a.value, pose["z"]) + C.BLADE_PLANE_OFS
                        b1 = max(a.value, pose["z"]) + C.BLADE_PLANE_OFS + C.WAFER_T
                        for k in range(1, C.SLOT_COUNT + 1):
                            if k != cur["n"] and b0 < C.slot_z(k) + C.WAFER_T and b1 > C.slot_z(k):
                                problems.append(f"{st.key}: Z move at input crosses slot {k} while carrying")
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
    for n, ng in ((1, False), (2, True), (4, False), (5, False)):
        cur["n"] = n
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
        if mode == "high":
            assert lo >= C.STATIONS["inspect"] - 1e-6 and hi <= C.STATIONS["input"] + 1e-6, (key, a, b)
            w0, w1 = z + C.BLADE_PLANE_OFS, z + C.BLADE_PLANE_OFS + C.WAFER_T
            assert w1 <= C.CRADLE_BAND_Z[0] - C.HIGH_SWING_MARGIN or w0 >= C.CRADLE_BAND_Z[1] + C.HIGH_SWING_MARGIN, (key, z)
        if mode == "carry":
            assert lo >= 180 - 1e-6 and hi <= 270 + 1e-6 and z == C.CASS_Z_CARRY, (key, a, b, z)
        if mode == "swing":
            assert z == C.ZC_SWING, (key, z)
    assert any(r[3] == "carry" and r[2] == 180 for r in rots) and any(r[3] == "carry" and r[2] == 270 for r in rots)
    assert sum(1 for r in rots if r[3] == "high") == 7          # I1 x 4 슬롯 + R3 x 정상 3 장
    # 사용자 요청: ① 에서 뺀 뒤 Z105 로 내렸다 올리지 않고 그대로 ② 로, UV 뒤에도 그대로 ① 로(한 방향)
    tz = [C.transfer_z(n) for n in range(1, C.SLOT_COUNT + 1)]
    assert tz == [C.side_z(1), C.side_z(2), C.side_z(3), 171.5, C.side_z(5)], tz       # 슬롯 4 만 ② 크래들 밑으로 15.5 mm
    for n in range(1, C.SLOT_COUNT + 1):
        i1 = Q.inspect_steps(n)[0]
        assert [a.kind for a in i1.actions] == ["z", "vz", "rot", "expect_th"] and i1.actions[0].value == tz[n - 1]
        assert C.ZC_SWING not in [a.value for a in i1.actions if a.kind == "z"]           # Z105 로 내리지 않음
    i1 = [(a, b, z) for k, a, b, m, z in rots if k == "I1"]
    assert i1 and all(a == C.STATIONS["input"] and b == C.STATIONS["inspect"] and z != C.ZC_SWING for a, b, z in i1), i1
    ret = [(a, b) for k, a, b, m, z in rots if k in ("R1", "R3")]
    assert ret and all(b > a for a, b in ret) and ret[-1][1] == C.STATIONS["input"], ret
    print(f"  OK walk {len(rots)} rotations: interlock passes, high swing inspect<->input outside the cradle band, carry 180-270 @ Z241, swing @ Z105")
    print(f"  OK no stop at Z105: pick 180 -> 120 at transfer height {tz} (I1 one rotation), return uv -> 120 (raise) -> 180")
    print("  OK Z move at input while carrying never crosses another slot, UV zone respected, UV lamp at ZC<=110 & R retracted, S4-none after retract")

    for z in (133, 151, 169, 171.5, 205):
        assert Q.can_rotate(z, True, 180, 120, "high")[0] and Q.can_rotate(z, True, 120, 180, "high")[0], z
    assert "CRADLE_BAND" in Q.can_rotate(187, True, 180, 120, "high")[1]           # 슬롯 4 들기 높이: 패드 위 0.5 mm
    assert "CRADLE_BAND" in Q.can_rotate(172, True, 180, 120, "high")[1]           # 팔 밑 여유 5 mm 미만
    assert not Q.can_rotate(205, False, 180, 120, "high")[0]
    assert "HIGH_PATH" in Q.can_rotate(205, True, 180, 220, "high")[1]             # 예전 옆길 쪽
    assert "HIGH_PATH" in Q.can_rotate(205, True, 120, 60, "high")[1]
    assert "CASS_HIGH" in Q.can_rotate(205, True, 180, 120, "high", carrying_cassette=True)[1]
    assert "MODE" in Q.can_rotate(205, True, 180, 150, "side")[1]                 # 옆길 모드는 없앰
    assert "Z_NOT_SWING" in Q.can_rotate(121.5, True, 0, 120, "swing")[1]
    assert "CASS_SWING" in Q.can_rotate(105, True, 180, 270, "swing", carrying_cassette=True)[1]
    assert "CARRY_PATH" in Q.can_rotate(241, True, 180, 0, "carry")[1]
    assert "THETA_RANGE" in Q.can_rotate(105, True, 0, -60, "swing")[1]
    assert Q.can_rotate(183, True, 120, 120.5, "fine")[0] and not Q.can_rotate(183, True, 120, 120.6, "fine")[0]
    assert not Q.can_rotate(192, True, 120, 120.2, "fine")[0]
    print("  OK can_rotate: high/carry/fine/swing rules + R retract required")

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

    # 동작 과정 그림(docs/process_flow.svg): 시간 = 시뮬 실측, 웨이퍼 단계 키를 모두 담음, 커밋된 파일 = 다시 만든 결과
    from wafer_cell import flow_diagram as F
    from wafer_cell.sim import CellSim

    def cycles(n_wafers, ng=False):
        s = CellSim(seed=1)
        s.ng_rate = 0.0
        s.command("home")
        for _ in range(200000):
            if s.state != "HOMING":
                break
            s.tick(0.05)
        s.command("start")
        for _ in range(400000):
            if len(s.cycles) >= n_wafers:
                break
            s.ng_next = ng
            s.tick(0.05)
        return s.cycles
    ok = cycles(C.SLOT_COUNT)
    for n in (1, C.SLOT_COUNT):
        assert abs(ok[n - 1] - F.wafer_time(n)) < 0.2, (n, ok[n - 1], F.wafer_time(n))
    ng = cycles(1, ng=True)
    assert abs(ng[0] - F.wafer_time(1, ok=False, k=C.SLOT_COUNT)) < 0.2, (ng[0], F.wafer_time(1, ok=False))
    wafer_keys = {st.key for st in Q.input_steps(1) + Q.empty_steps(1) + Q.inspect_steps(1) + Q.ng_steps(5)
                  + Q.uv_steps() + Q.return_steps(1) + Q.cart_in_steps(None) + Q.cart_out_steps(None)}
    assert wafer_keys <= set(F.covered_keys()), wafer_keys - set(F.covered_keys())
    svg = F.build_svg()
    assert svg == F.build_svg()                                   # 같은 입력 = 같은 글자
    for n in range(1, C.SLOT_COUNT + 1):
        F.build_svg(n)
    committed = F.OUT_DEFAULT.read_text(encoding="utf-8").replace("\r", "")
    assert committed == svg, "docs/process_flow.svg is stale: run py -3.13 wafer_cell/flow_diagram.py"
    print(f"  OK flow diagram: wafer time = sim (slot 1 {ok[0]:.1f}s, slot {C.SLOT_COUNT} {ok[-1]:.1f}s, NG {ng[0]:.1f}s),"
          " all step keys covered, committed SVG up to date")
    print("OK (all passed)")


if __name__ == "__main__":
    main()
