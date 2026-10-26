"""웨이퍼 검사셀 R22 동작 과정 그림(한 장 스토리보드) 생성기 -> wafer_cell/docs/process_flow.svg

순수 로직 + 표준 라이브러리만(cv2 · numpy 불필요). 그림의 자세 · 경로 · 시간은 손으로 그리지 않고
sequence.py 단계의 동작(MoveZ · MoveR · Rotate · Capture · UvExpose)과 cell_config.py 수치에서 뽑는다.
  - 시간 = 동작 거리 / 가정 속도(등속, 시뮬과 같은 값). 확인 동작(S4 · TOF · θ 자리 · Sharp)은 0 s.
  - 옆 그림(r-z 단면) 치수는 web/app.js buildElev 와 같은 R9 몸체 값(가정).
자리 · 단계 · 높이를 바꾸면 다시 실행한다(tests/test_wafer_sequence.py 가 커밋된 SVG 와 비교).

실행: py -3.13 wafer_cell/flow_diagram.py [--slot 1] [--ng-slot 5] [--out wafer_cell/docs/process_flow.svg]
"""
import argparse
import math
import pathlib
import sys
from xml.sax.saxutils import escape

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from wafer_cell import cell_config as C  # noqa: E402
from wafer_cell import sequence as S  # noqa: E402

OUT_DEFAULT = C.CELL_DIR / "docs" / "process_flow.svg"
FONT = "'Malgun Gothic','Apple SD Gothic Neo','Noto Sans CJK KR','Noto Sans KR','WenQuanYi Zen Hei',sans-serif"

# 색 (흰 바탕 · 인쇄용)
INK, INK2, INK3 = "#0f172a", "#334155", "#64748b"
AX = {"rot": "#2563eb", "z": "#ea580c", "r": "#059669", "capture": "#b45309", "uv": "#7c3aed"}
LANE = {"common": "#4f46e5", "ok": "#15803d", "ng": "#b91c1c", "sum": "#64748b"}
WAFER = {"u": ("#ddd6fe", "#6d28d9"), "ok": ("#bbf7d0", "#15803d"), "ng": ("#fecaca", "#b91c1c")}
PART = ("#e2e8f0", "#64748b")
NOGO = ("#fed7aa", "#ea580c")     # 들어가면 안 되는 높이(② 크래들 띠)
OKZONE = ("#cffafe", "#0891b2")   # 그 높이 안에서만(UV 탑 밑)
ROBOT = ("#cbd5e1", "#475569")

ST_NAME = {"input": "① 투입", "inspect": "② 검사", "ng": "③ 불량", "uv": "UV 탑", "cart": "대차"}
START_POSE = (C.STATIONS["input"], C.ZC_SWING, C.RX_RET)       # 웨이퍼 한 장의 시작 = ① 앞 · Z105 · R 후진

# 판 크기
W, H = 2480, 1780
COLX = [150 + i * 388 for i in range(6)]
FW, FH = 360, 320
ROW_Y = [118, 474, 830, 1186]       # 0: 지도 · 범례 · 규칙, 1: 공통, 2: OK, 3: NG · 다음 · B · 표
TIME_Y = 1542


def g(v):
    """숫자 글자(끝의 .0 없앰)."""
    return f"{v:g}"


def est_w(s, size):
    """글자 폭 어림(px): 한글 · 기호 1 em, 그 밖 0.58 em."""
    return sum(size * (1.0 if ord(ch) >= 0x1100 else 0.58) for ch in s)


# ------------------------------------------------------------ 동작 걷기
def walk(actions, pose):
    """동작 목록을 등속으로 걸어 [(종류, 시작 자세, 끝 자세, 초, 글자)] 와 끝 자세를 돌려준다.
    자세 = (θ, Z, RX). 보정(corr)은 δ 0 으로 본다. 확인 동작은 앞 항목 글자에 붙인다."""
    th, z, r = pose
    items, pre = [], []

    def tag(s):
        if items:
            k, p0, p1, t, txt = items[-1]
            items[-1] = (k, p0, p1, t, txt + s)
        else:
            pre.append(s.strip())

    for a in actions:
        p0 = (th, z, r)
        if a.kind == "z":
            if abs(a.value - z) > 1e-9:
                t = abs(a.value - z) / C.Z_SPEED
                z = a.value
                items.append(("z", p0, (th, z, r), t, f"Z {g(z)}"))
        elif a.kind == "r":
            if abs(a.value - r) > 1e-9 or a.corr:
                t = abs(a.value - r) / C.R_SPEED
                r = a.value
                txt = "R 후진" if r == C.RX_RET else f"R {g(r)}"
                items.append(("r", p0, (th, z, r), t, txt + ("+δr" if a.corr else "")))
        elif a.kind == "rot":
            if abs(a.value - th) > 1e-9:
                t = abs(a.value - th) / C.THETA_SPEED
                items.append(("rot", p0, (a.value, z, r), t, f"θ {g(th)}→{g(a.value)}"))
                th = a.value
            elif a.corr:
                items.append(("rot", p0, p0, 0.0, "θ+δθ"))
        elif a.kind == "capture":
            items.append(("capture", p0, p0, C.CAPTURE_S, "촬영 · 판정"))
        elif a.kind == "uv":
            items.append(("uv", p0, p0, C.UV_S, f"UV {g(C.UV_S)} s"))
        elif a.kind == "check_pick":
            tag(" (S4)")
        elif a.kind == "expect":
            name = {"S4": "S4", "TOF_IN": "TOF-IN", "TOF_NG": "TOF-NG"}.get(a.sensor, a.sensor)
            if a.sensor == "S4":
                tag(" (S4)" if a.expect else " (S4 없음)")
            else:
                tag(f" ({name}{'' if a.expect else ' 없음'})")
    return items, (th, z, r), pre


def steps_time(steps, pose):
    tot = 0.0
    for st in steps:
        items, pose, _ = walk(st.actions, pose)
        tot += sum(i[3] for i in items)
    return tot, pose


def wafer_time(n, ok=True, k=C.SLOT_COUNT):
    """웨이퍼 한 장(① 슬롯 n) 시간: 정상 = 꺼내기 · 검사 · UV · 되돌리기 / 불량 = 꺼내기 · 검사 · ③ 슬롯 k."""
    steps = S.input_steps(n) + S.inspect_steps(n) + (S.uv_steps() + S.return_steps(n) if ok else S.ng_steps(k))
    return steps_time(steps, START_POSE)[0]


def frames(n, k):
    """칸 목록. 키 = 칸 번호, 단계, 옆 그림 자리, 줄, 시작 · 끝에 포크가 웨이퍼를 들었는지, 자리 상태."""
    by = {st.key: st for st in (S.input_steps(n) + S.inspect_steps(n) + S.uv_steps() + S.return_steps(n)
                                + S.ng_steps(k))}
    tz, sz = C.transfer_z(n), C.side_z(n)
    lo, hi = C.CRADLE_BAND_Z
    sec = lambda *ks: [by[x] for x in ks]   # noqa: E731
    m = C.HIGH_SWING_MARGIN
    swing_note = (f"띠({g(lo)}~{g(hi)} ± {g(m)})에 걸려 Z {g(tz)} 로 내린 뒤 돈다" if tz != sz
                  else f"띠({g(lo)}~{g(hi)} ± {g(m)}) 밖이라 들기 높이 그대로 돈다")
    f = [
        dict(no="1", lane="common", steps=sec("P1", "P2", "P3"), side="input", title=f"① 슬롯 {n} 꺼내기",
             carry=(None, "u"), ins=("full", "taken"),
             note=f"S4 OFF = 빈 슬롯 → E1 (R 후진 · Z {g(C.ZC_SWING)}) 다음 슬롯"),
        dict(no="2", lane="common", steps=sec("I1"), side="inspect", title="② 로 높이 선회", carry=("u", "u"), band=True,
             note=swing_note),
        dict(no="3", lane="common", steps=sec("I2"), side="inspect", title="② 크래들에 놓기", carry=("u", None), cradle="u",
             note=f"패드 {g(C.CRADLE_SEAT_Z)} 위로 들어가 Z {g(C.CRADLE_ZC_SEAT)} 로 내려놓음"),
        dict(no="4", lane="common", steps=sec("I3"), side="inspect", title="촬영 · 판정 (PC)", carry=(None, None), cradle="u",
             cam=True, note=f"OK / NG + 치우침 δr · δθ, {g(C.JUDGE_TIMEOUT_S)} s 안에 답"),
        dict(no="5", lane="common", steps=sec("I4"), side="inspect", title="보정해서 다시 집기", carry=(None, "u"),
             note=f"창 δr {g(C.CORR_DR[0])} ~ +{g(C.CORR_DR[1])} · |δθ| ≤ {g(C.CORR_DTH)}° (Z {g(C.CRADLE_ZC_SEAT)} 에서)"),
        dict(no="6", lane="ok", steps=sec("U1"), side="uv", title="UV 탑으로", carry=("ok", "ok"),
             note=f"Z {g(C.ZC_SWING)} · R 후진 에서 선회"),
        dict(no="7", lane="ok", steps=sec("U2", "U3", "U4", "U5"), side="uv", title="UV 받침에 넘기기",
             carry=("ok", None), uvseat="ok", uvzone=True,
             note=f"탑 밑(RX > {g(C.RX_RET_OK)})은 ZC {g(C.UV_ZC_HAND)} ~ {g(C.UV_ZC_IN)} 만"),
        dict(no="8", lane="ok", steps=sec("U6"), side="uv", title=f"UV 조사 {g(C.UV_S)} s", carry=(None, None),
             uvseat="ok", lamp=True, note=f"ZC ≤ {g(C.UV_ZC_LAMP_MAX)} · 켜기 7 조건 · A_UV {g(C.UV_AUV_S)} s 안"),
        dict(no="9", lane="ok", steps=sec("U7"), side="uv", title="UV 받침에서 다시 집기", carry=(None, "ok"),
             note="S4 ON 확인 뒤 빼고 내림"),
        dict(no="10", lane="ok", steps=sec("R1", "R2"), side="inspect", title=f"② 앞으로 · 슬롯 {n} 높이",
             carry=("ok", "ok"), band=True, note="UV 탑 밑은 높이 제한 → 높이는 ② 앞에서 맞춤"),
        dict(no="11", lane="ok", steps=sec("R3", "R4", "R5"), side="input", title=f"① 슬롯 {n} 에 되돌리기",
             carry=("ok", None), ins=("taken", "back"),
             note=(f"① 앞에서 넣기 높이 Z {g(sz)} 로 올린 뒤 넣음" if tz != sz else "높이 그대로 ① 로 → 원래 슬롯에 넣음")),
        dict(no="6'", lane="ng", steps=sec("N1"), side="ng", title="③ 불량으로", carry=("ng", "ng"),
             note=f"Z {g(C.ZC_SWING)} · UV 탑({g(C.STATIONS['uv'])}°) 앞을 지남"),
        dict(no="7'", lane="ng", steps=sec("N2"), side="ng", title=f"③ 슬롯 {k} 에 넣기", carry=("ng", None), ngk=k,
             note="위 슬롯부터 채움 · ③ 가득 → 새 카세트 대기"),
    ]
    # 공통 -> OK 줄은 이어서, NG 줄은 5 번 칸(다시 집기) 끝 자세에서 갈라짐
    pose, fork = START_POSE, None
    for fr in f:
        if fr["lane"] == "ng" and fork is not None:
            pose, fork = fork, None
        acts = [a for st in fr["steps"] for a in st.actions]
        fr["items"], end, fr["pre"] = walk(acts, pose)
        fr["start"], fr["end"] = pose, end
        fr["t"] = sum(i[3] for i in fr["items"])
        fr["keys"] = [st.key for st in fr["steps"]]
        if fr["no"] == "5":
            fork = end
        pose = end
    return f


def covered_keys(n=1, k=C.SLOT_COUNT):
    """그림이 다루는 단계 키(칸 + 요약 칸 + 다음 웨이퍼 상자)."""
    keys = [x for fr in frames(n, k) for x in fr["keys"]]
    keys += [st.key for st in S.cart_in_steps(None) + S.cart_out_steps(None) + S.empty_steps(n)]
    return keys


# ------------------------------------------------------------ SVG 조각
class Svg:
    def __init__(self):
        self.out = []
        self.clip_n = 0

    def add(self, s):
        self.out.append(s)

    def text(self, x, y, s, size=13, weight=400, fill=INK, anchor="start", extra=""):
        self.add(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" font-weight="{weight}" fill="{fill}" '
                 f'text-anchor="{anchor}"{extra}>{escape(s)}</text>')

    def rect(self, x, y, w, h, fill="none", stroke="none", sw=1.0, rx=0, extra=""):
        self.add(f'<rect x="{x:.1f}" y="{y:.1f}" width="{max(w, 0):.1f}" height="{max(h, 0):.1f}" rx="{rx}" '
                 f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{extra}/>')

    def line(self, x1, y1, x2, y2, color, sw=2.0, arrow=None, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        m = f' marker-end="url(#ah-{arrow})"' if arrow else ""
        self.add(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{color}" '
                 f'stroke-width="{sw}"{d}{m}/>')

    def poly(self, pts, color, sw=2.0, arrow=None, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        m = f' marker-end="url(#ah-{arrow})"' if arrow else ""
        p = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        self.add(f'<polyline points="{p}" fill="none" stroke="{color}" stroke-width="{sw}" '
                 f'stroke-linejoin="round"{d}{m}/>')

    def badge(self, x, y, label, color, r=8.5, size=10.5):
        self.add(f'<g class="badge"><circle cx="{x:.1f}" cy="{y:.1f}" r="{r}" fill="{color}" stroke="#ffffff" '
                 f'stroke-width="1.2"/>')
        self.text(x, y + size * 0.36, label, size=size, weight=700, fill="#ffffff", anchor="middle")
        self.add("</g>")


def markers():
    out = ["<defs>"]
    cols = dict(AX)
    cols.update({f"l{k}": v for k, v in LANE.items()})
    for k, c in cols.items():
        out.append(f'<marker id="ah-{k}" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="5.5" markerHeight="5.5" '
                   f'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>')
    out.append("</defs>")
    return out


def arc_path(cx, cy, r, a0, a1):
    """θ a0 -> a1 호(위 = θ0, 반시계 = +, 화면 왼쪽으로)."""
    def pt(a):
        return cx - r * math.sin(math.radians(a)), cy - r * math.cos(math.radians(a))
    x0, y0 = pt(a0)
    x1, y1 = pt(a1)
    large = 1 if abs(a1 - a0) > 180 else 0
    sweep = 0 if a1 > a0 else 1
    return f"M{x0:.1f},{y0:.1f} A{r:.1f},{r:.1f} 0 {large} {sweep} {x1:.1f},{y1:.1f}"


def polar(cx, cy, r, a):
    return cx - r * math.sin(math.radians(a)), cy - r * math.cos(math.radians(a))


# ------------------------------------------------------------ 옆 그림 (r-z 단면)
class Side:
    R_LEFT, Z_TOP, SC = 20.0, 306.0, 340.0 / 320.0

    def __init__(self, svg, x, y, w, h):
        self.s, self.x, self.y, self.w, self.h = svg, x, y, w, h

    def sx(self, r):
        return self.x + (r - self.R_LEFT) * self.SC

    def sy(self, z):
        return self.y + (self.Z_TOP - z) * self.SC

    def box(self, r0, z0, w, h, fill, stroke, sw=0.8, extra=""):
        self.s.rect(self.sx(r0), self.sy(z0 + h), w * self.SC, h * self.SC, fill, stroke, sw, extra=extra)

    def cassette(self, z0, dx):
        o = z0 - 172.0
        f, s = PART
        self.box(272 + dx, z0, 20, 118, "#cbd5e1", s)
        self.box(215 + dx, z0, 85, 8, f, s)
        self.box(215 + dx, z0 + 8, 71, 10, f, s)
        for kk in range(2, C.SLOT_COUNT + 1):
            self.box(215 + dx, C.slot_z(kk) - C.LEDGE_T + o, 71, C.LEDGE_T, f, s)
        self.box(215 + dx, C.CASS_TOP_Z + o, 77, 4, f, s)

    def wafer(self, r0, z0, kind, extra=""):
        fill, stroke = WAFER[kind]
        self.box(r0, z0, C.WAFER_D, C.WAFER_T, fill, stroke, 1.0, extra=extra)

    def station(self, name, fr, n):
        dx = C.INPUT_DOCK_SHIFT if name == "input" else 0.0
        if name in ("input", "ng"):
            self.cassette(172.0, dx)
            self.box(228.5 + dx, 140, 14, 6, "#fde68a", "#b45309")              # ToF
            if name == "input":
                self.box(205 + dx, 172 + C.CASS_PLATE_UNDER, 100, 6, "#e0e7ff", "#6366f1")   # 클램프 윗판
                st0, st1 = fr.get("ins", ("taken", "taken"))
                for kk in range(1, C.SLOT_COUNT + 1):
                    if kk != n:
                        self.wafer(185.5 + dx, C.slot_z(kk), "u")
                    elif st1 == "full":
                        self.wafer(185.5 + dx, C.slot_z(kk), "u")
                    elif st1 == "back":
                        self.wafer(185.5 + dx, C.slot_z(kk), "ok")
            else:
                kk = fr.get("ngk")
                if kk:
                    self.wafer(185.5, C.slot_z(kk), "ng")
        elif name == "inspect":
            f, s = PART
            lo, hi = C.CRADLE_BAND_Z
            if fr.get("band"):
                m = C.HIGH_SWING_MARGIN
                self.box(205, lo - m, 125, hi - lo + 2 * m, NOGO[0], NOGO[1], 0.8,
                         extra=' fill-opacity="0.55" stroke-dasharray="4 3"')
            self.box(276, 60, 20, 184, "#cbd5e1", s)
            self.box(262, 244, 56, 6, f, s)
            self.box(220, 244, 45, 6, f, s)
            self.box(218, 250, 8, 2, "#94a3b8", s)
            self.box(276.5, 250, 8, 2, "#94a3b8", s)
            self.box(238, 212, 30, 27, "#fef3c7", "#d97706")                      # 백라이트
            if fr.get("cradle"):
                self.wafer(185.5, C.CRADLE_SEAT_Z, fr["cradle"])
            if fr.get("cam"):
                cx = self.sx(235.5)
                self.s.line(cx, self.sy(290), cx, self.sy(C.CRADLE_SEAT_Z + 4), AX["capture"], 1.4, dash="4 3")
                self.s.rect(cx - 14, self.sy(302), 28, 11, "#1e293b", "#0f172a", 1, rx=2)
                self.s.rect(cx - 4, self.sy(291), 8, 4, "#475569", "none")
        elif name == "uv":
            sz, ux = C.UV_SEAT_Z, C.station_r("uv") - 50.0
            f, s = PART
            if fr.get("uvzone"):
                self.box(ux - 10, C.UV_ZC_HAND + C.BLADE_PLANE_OFS, 130, C.UV_ZC_IN - C.UV_ZC_HAND, OKZONE[0], OKZONE[1],
                         0.8, extra=' fill-opacity="0.7" stroke-dasharray="4 3"')
            self.box(ux - 6, sz - 3, 8, 4, "#94a3b8", s)
            self.box(ux + 104, sz - 3, 8, 4, "#94a3b8", s)
            self.box(ux + 108, sz - 63, 12, 173, "#cbd5e1", s)
            on = fr.get("lamp")
            self.box(ux + 4, sz + 4, 92, 30, "#8b5cf6" if on else "#f5f3ff", "#7c3aed", 0.8,
                     extra=f' fill-opacity="{0.55 if on else 0.6}"')
            self.box(ux - 4, sz + 34, 108, 40, "#ede9fe", "#7c3aed")
            if fr.get("uvseat"):
                self.wafer(ux, sz, fr["uvseat"])

    def robot(self, z, rx, carry, ghost=False):
        f, s = ROBOT
        d = z - C.ZC_SWING
        ex = ' opacity="0.3"' if ghost else ""
        self.s.add(f"<g{ex}>")
        self.box(-50, 105 + d, 100, 30, f, s)
        self.box(-10, 135 + d, 48, 20, f, s)
        self.box(38, 138 + d, max(4.0, 58 + rx), 14, "#e2e8f0", s)
        self.box(96 + rx, 139 + d, 11, 12, f, s)
        self.box(107 + rx, 151 + d, 34, 14, f, s)
        self.box(107 + rx, 165 + d, 96, 4, "#475569", "#334155")
        if carry:
            self.wafer(134.5 + rx, C.BLADE_PLANE_OFS + z, carry)
        self.s.add("</g>")

    def pt(self, pose):
        """포크 위 웨이퍼 자리 중심(화면)."""
        _, z, rx = pose
        return self.sx(C.BLADE_ORIGIN_X + C.WAFER_ON_BLADE_X + rx), self.sy(z + C.BLADE_PLANE_OFS + C.WAFER_T / 2)


# ------------------------------------------------------------ 칸
def seg_dist(p, a, b):
    """점 p 와 선분 ab 사이 거리(px)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def chain_lines(items, pre, max_w, size=12.5):
    """동작 글자를 (번호, 색, 글자) 조각으로 줄 나눔."""
    pieces = [(None, INK3, p) for p in pre] + [(str(i + 1), AX[it[0]], it[4]) for i, it in enumerate(items)]
    lines, cur, w = [], [], 0.0
    for num, col, txt in pieces:
        pw = est_w(txt, size) + (est_w(num, size) + 6 if num else 0) + 12
        if cur and w + pw > max_w:
            lines.append(cur)
            cur, w = [], 0.0
        cur.append((num, col, txt))
        w += pw
    if cur:
        lines.append(cur)
    return lines


def emit_chain(svg, x, y, line, size=12.5):
    parts = []
    for i, (num, col, txt) in enumerate(line):
        sep = "   " if i else ""
        if num:
            parts.append(f'<tspan font-weight="700" fill="{col}">{sep}{escape(num)}</tspan>'
                         f'<tspan fill="{INK2}"> {escape(txt)}</tspan>')
        else:
            parts.append(f'<tspan fill="{INK3}">{sep}{escape(txt)}</tspan>')
    svg.add(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" xml:space="preserve">{"".join(parts)}</text>')


def dial(svg, cx, cy, r, th0, th1, highlight, lane_col):
    svg.add(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r}" fill="#f8fafc" stroke="#cbd5e1" stroke-width="1"/>')
    for name, a in C.STATIONS.items():
        x, y = polar(cx, cy, r, a)
        on = name == highlight
        svg.add(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{3.6 if on else 2.4}" fill="{lane_col if on else "#94a3b8"}"/>')
    if abs(th1 - th0) > 1e-6:
        svg.add(f'<path d="{arc_path(cx, cy, r * 0.62, th0, th1)}" fill="none" stroke="{AX["rot"]}" stroke-width="2.4" '
                f'marker-end="url(#ah-rot)"/>')
    x, y = polar(cx, cy, r * 0.62, th1)
    svg.line(cx, cy, x, y, INK2, 1.6)
    svg.add(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="2.6" fill="{INK2}"/>')


def frame_box(svg, x, y, w, h, no, title, keys, lane):
    col = LANE[lane]
    svg.add('<g class="node">')
    svg.rect(x, y, w, h, "#ffffff", col, 1.6, rx=8)
    svg.badge(x + 24, y + 22, no, col, r=15, size=14 if len(no) < 3 else 12)
    svg.text(x + 46, y + 28, title, size=16, weight=700, fill=INK)
    if keys:
        svg.text(x + w - 12, y + 27, keys, size=12, fill=INK3, anchor="end")


def key_span(keys):
    return keys[0] if len(keys) == 1 else f"{keys[0]}–{keys[-1]}"


def draw_frame(svg, x, y, fr, n):
    lane = fr["lane"]
    frame_box(svg, x, y, FW, FH, fr["no"], fr["title"], key_span(fr["keys"]), lane)
    vx, vy, vw, vh = x + 10, y + 44, FW - 20, 180
    svg.clip_n += 1
    cid = f"cv{svg.clip_n}"
    svg.add(f'<clipPath id="{cid}"><rect x="{vx}" y="{vy}" width="{vw}" height="{vh}"/></clipPath>')
    svg.rect(vx, vy, vw, vh, "#f8fafc", "#e2e8f0", 1, rx=4)
    svg.add(f'<g clip-path="url(#{cid})">')
    sv = Side(svg, vx, vy, vw, vh)
    sv.station(fr["side"], fr, n)
    (t0, z0, r0), (t1, z1, r1) = fr["start"], fr["end"]
    c0, c1 = fr["carry"]
    if abs(z0 - z1) > 1e-6 or abs(r0 - r1) > 1e-6:
        sv.robot(z0, r0, c0, ghost=True)
    sv.robot(z1, r1, c1)
    # 이동 경로 + 순번. 같은 선을 오르내리는 Z 는 위로 = 왼쪽, 아래로 = 오른쪽으로 3 px 띄움
    segs = []
    for i, it in enumerate(fr["items"]):
        kind, p0, p1 = it[0], it[1], it[2]
        a, b = sv.pt(p0), sv.pt(p1)
        if kind == "z":
            off = -3.0 if b[1] < a[1] else 3.0
            a, b = (a[0] + off, a[1]), (b[0] + off, b[1])
        segs.append((i, kind, a, b))
    for i, kind, a, b in segs:
        if kind in ("z", "r"):
            svg.line(a[0], a[1], b[0], b[1], AX[kind], 2.4, arrow=kind)
    placed = []
    for i, kind, a, b in segs:
        cands = []
        if kind in ("z", "r"):
            for t in (0.5, 0.3, 0.7):
                mx, my = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
                if kind == "z":
                    cands += [(mx - 13, my), (mx + 13, my)]
                else:
                    cands += [(mx, my - 12), (mx, my + 12)]
        else:
            cands = [(a[0] - 64, a[1] - 16), (a[0] - 64, a[1] + 16), (a[0] - 96, a[1] - 16)]
        others = [sg for sg in segs if sg[0] != i and sg[1] in ("z", "r")]
        best, score = cands[0], -1e9
        for c in cands:
            d = min([seg_dist(c, sg[2], sg[3]) for sg in others] + [40.0])
            d = min([d] + [math.hypot(c[0] - q[0], c[1] - q[1]) - 9 for q in placed])
            if c[0] < vx + 10 or c[0] > vx + vw - 10 or c[1] < vy + 10 or c[1] > vy + vh - 10:
                d -= 30
            if fr.get("band") and c[1] < vy + 30:      # 왼쪽 위 틈 글자 자리
                d -= 30
            if d > score + 0.5:
                best, score = c, d
        placed.append(best)
        svg.badge(best[0], best[1], str(i + 1), AX[kind])
    if fr.get("band"):
        # 운반 웨이퍼와 ② 크래들 띠 사이 틈: 겹치는 반경(205~232.5) 안에 치수선, 글은 왼쪽 위
        lo, hi = C.CRADLE_BAND_Z
        bot = z1 + C.BLADE_PLANE_OFS
        top = bot + C.WAFER_T
        za, zb, where = (top, lo, "밑") if top <= lo else (hi, bot, "위")
        gx = sv.sx(216.0)
        svg.line(gx, sv.sy(za), gx, sv.sy(zb), INK2, 1.2)
        for zz in (za, zb):
            svg.line(gx - 4, sv.sy(zz), gx + 4, sv.sy(zz), INK2, 1.2)
        svg.text(vx + 8, vy + 18, f"② 크래들 띠 {where} 틈 {g(round(zb - za, 1))} mm", size=11.5, weight=700, fill=AX["z"])
    svg.add("</g>")
    # 아래: θ 다이얼 + 동작 글자
    dial(svg, x + 36, y + 268, 23, t0, t1, fr["side"], LANE[lane])
    tx = x + 72
    lines = chain_lines(fr["items"], fr["pre"], FW - 84, size=12)
    yy = y + 244
    for ln in lines:
        emit_chain(svg, tx, yy, ln, size=12)
        yy += 16
    if fr.get("note"):
        svg.text(tx, yy - 1, fr["note"], size=11.5, fill=INK3)
        yy += 15
    svg.text(tx, y + FH - 10, f"{ST_NAME[fr['side']]} θ{g(C.STATIONS[fr['side']])} · 약 {fr['t']:.1f} s",
             size=12, weight=700, fill=LANE[lane])
    svg.add("</g>")
    return yy


def summary_frame(svg, x, y, no, title, steps, pose, th_path, extra):
    frame_box(svg, x, y, FW, FH, no, title, key_span([st.key for st in steps]), "sum")
    tot = 0.0
    yy = y + 64
    for st in steps:
        items, pose, pre = walk(st.actions, pose)
        tot += sum(i[3] for i in items)
        txt = " → ".join(i[4] for i in items) or "-"
        if pre:
            txt = " ".join(pre) + " " + txt
        svg.text(x + 16, yy, st.key, size=12, weight=700, fill=INK2)
        svg.text(x + 46, yy, txt, size=12, fill=INK2)
        yy += 19
    cx, cy, r = x + 56, y + 238, 32
    svg.add(f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="#f8fafc" stroke="#cbd5e1" stroke-width="1"/>')
    for name, a in C.STATIONS.items():
        px, py = polar(cx, cy, r, a)
        on = name in ("input", "cart")
        svg.add(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="{3.6 if on else 2.4}" fill="{LANE["sum"] if on else "#cbd5e1"}"/>')
    a0, a1 = th_path
    svg.add(f'<path d="{arc_path(cx, cy, r * 0.62, a0, a1)}" fill="none" stroke="{AX["rot"]}" stroke-width="2.6" '
            f'marker-end="url(#ah-rot)"/>')
    lx, ly = polar(cx, cy, r + 12, C.STATIONS["input"])
    svg.text(lx, ly + 5, "①", size=12, fill=INK2, anchor="middle")
    lx, ly = polar(cx, cy, r + 7, C.STATIONS["cart"])
    svg.text(lx, ly + 4, "대차", size=11.5, fill=INK2)
    yy = y + 220
    for ln in extra:
        svg.text(x + 140, yy, ln, size=12, fill=INK2)
        yy += 19
    svg.text(x + 16, y + FH - 10, f"카세트마다 1 번 · 약 {tot:.1f} s", size=12, weight=700, fill=LANE["sum"])
    svg.add("</g>")
    return pose


# ------------------------------------------------------------ 판 전체
def build_svg(n=1, k=C.SLOT_COUNT):
    assert 1 <= n <= C.SLOT_COUNT and 1 <= k <= C.SLOT_COUNT
    fs = frames(n, k)
    svg = Svg()
    svg.add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
            f'font-family="{FONT}">')
    svg.add("<!-- generated by wafer_cell/flow_diagram.py - do not edit by hand -->")
    svg.out += markers()
    svg.rect(0, 0, W, H, "#ffffff")
    st = C.STATIONS
    svg.text(30, 56, f"웨이퍼 검사셀 {C.CELL} — 동작 과정 (웨이퍼 1 장, ① 슬롯 {n})", size=32, weight=700)
    svg.text(30, 88, f"자리: ③ 불량 θ{g(st['ng'])} · UV 탑 θ{g(st['uv'])} · ② 검사 θ{g(st['inspect'])} · ① 투입 θ{g(st['input'])}"
                     f" · 대차 θ{g(st['cart'])}   |   사용자 변경 반영: ③ ↔ UV 자리, ① ↔ ② 높이 선회(옆길 없음)   |   "
                     f"시간 = 가정 속도 θ {g(C.THETA_SPEED)}°/s · Z {g(C.Z_SPEED)} · R {g(C.R_SPEED)} mm/s",
             size=15, fill=INK3)
    R0H = ROW_Y[1] - ROW_Y[0] - 36
    draw_map(svg, 30, ROW_Y[0], COLX[2] - 28 - 30, R0H, n, k)
    draw_legend(svg, COLX[2], ROW_Y[0], FW, R0H)
    draw_rules(svg, COLX[3], ROW_Y[0], COLX[5] + FW - COLX[3], R0H, n)

    # 1 줄: A + 공통 1~5
    summary_frame(svg, COLX[0], ROW_Y[1], "A", "대차 → ① 카세트", S.cart_in_steps(None),
                  (C.THETA_STANDBY, C.ZC_SWING, C.RX_RET), (C.STATIONS["cart"], C.STATIONS["input"]),
                  [f"카세트 든 선회 Z {g(C.CASS_Z_CARRY)}", f"θ {g(st['cart'])} → {g(st['input'])} 짧은 길", "(③ · UV 쪽 안 지남)"])
    rows = {"common": (1, 1), "ok": (2, 0), "ng": (3, 0)}
    pos = {}
    cnt = {"common": 0, "ok": 0, "ng": 0}
    for fr in fs:
        row, c0 = rows[fr["lane"]]
        x, y = COLX[c0 + cnt[fr["lane"]]], ROW_Y[row]
        cnt[fr["lane"]] += 1
        draw_frame(svg, x, y, fr, n)
        pos[fr["no"]] = (x, y)
    # 다음 웨이퍼 상자 · B · 높이 표
    nx, ny = COLX[2], ROW_Y[3]
    draw_next(svg, nx, ny, n)
    summary_frame(svg, COLX[3], ROW_Y[3], "B", "① 카세트 → 대차", S.cart_out_steps(None),
                  (C.STATIONS["input"], C.ZC_SWING, C.RX_RET), (C.STATIONS["input"], C.STATIONS["cart"]),
                  ["대차: 다음 받침으로", f"물러남 {g(C.CART_RETREAT)} · 옆걸음 {g(C.CART_LATERAL)} · 180°",
                   "다 하면 출발 → 새 대차"])
    draw_table(svg, COLX[4], ROW_Y[3], COLX[5] + FW - COLX[4], FH, n)
    draw_arrows(svg, pos, (nx, ny))
    draw_lane_labels(svg)
    draw_times(svg, fs, n, k)
    svg.text(30, H - 40, "※ 시간은 가정 속도 등속(가속 없음) · 확인 동작과 PC 판정 대기는 0 s 로 본 하한값입니다. 옆 그림 치수는 R9 몸체 · 가정값(화면과 같음), 틈은 계산값 [불명].",
             size=12.5, fill=INK3)
    svg.text(30, H - 18, "※ 이 그림은 wafer_cell/flow_diagram.py 가 sequence.py(단계 동작) · cell_config.py(자리 · 높이)에서 만듭니다. "
                         "자리 · 단계 · 높이를 바꾸면 다시 실행: py -3.13 wafer_cell/flow_diagram.py",
             size=12.5, fill=INK3)
    svg.add("</svg>")
    return "\n".join(svg.out) + "\n"


def draw_map(svg, x, y, w, h, n, k):
    svg.add('<g class="node">')
    svg.rect(x, y, w, h, "#ffffff", "#94a3b8", 1.2, rx=8)
    svg.text(x + 16, y + 28, "셀 지도 (위에서 봄 · 화면 평면도와 같은 방향)", size=16, weight=700)
    cx, cy, s = x + 200, y + 182, 0.4
    svg.add(f'<circle cx="{cx}" cy="{cy}" r="{85 * s:.1f}" fill="#e2e8f0" stroke="#64748b" stroke-width="1"/>')
    svg.text(cx, cy + 4, "로봇", size=11, fill=INK2, anchor="middle")
    fills = {"input": ("#ede9fe", "#6d28d9"), "inspect": ("#fef3c7", "#d97706"), "ng": ("#fee2e2", "#b91c1c"),
             "uv": ("#f3e8ff", "#7c3aed"), "cart": ("#f1f5f9", "#64748b")}
    lab = {"input": "1 · 11 · A · B", "inspect": "2–5 · 10", "uv": "6–9", "ng": "6' · 7'", "cart": "A · B"}
    for name, a in C.STATIONS.items():
        rr = C.station_r(name) * s
        px, py = polar(cx, cy, rr, a)
        f, st_ = fills[name]
        if name == "cart":
            svg.rect(px - 8, py - 52, 74, 104, f, st_, 1.2, rx=6)
        else:
            svg.add(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="{50 * s:.1f}" fill="{f}" stroke="{st_}" stroke-width="1.4"/>')
        if name == "cart":
            lx, ly, anc = px + 29, py - 6, "middle"
        else:
            lx, ly = polar(cx, cy, rr + 40, a)
            anc = "end" if lx < cx - 20 else ("start" if lx > cx + 20 else "middle")
            if name == "ng":
                lx, ly, anc = px + 26, py - 2, "start"
            if name == "input":
                lx, ly, anc = px + 26, py + 2, "start"
        svg.text(lx, ly, f"{ST_NAME[name]} θ{g(a)}", size=12.5, weight=700, fill=st_, anchor=anc)
        svg.text(lx, ly + 15, f"칸 {lab[name]}", size=11.5, fill=INK3, anchor=anc)
    # 웨이퍼 길(반경을 나눠 겹치지 않게) + 칸 번호
    paths = [
        (68, st_a("input"), st_a("inspect"), "common", "2", 150),
        (68, st_a("inspect"), st_a("uv"), "ok", "6", 90),
        (56, st_a("uv"), st_a("input"), "ok", "", None),
        (44, st_a("inspect"), st_a("ng"), "ng", "6'", 22),
        (68, st_a("input"), st_a("cart"), "sum", "A·B", 225),
    ]
    for r_px, a0, a1, lane, lb, at in paths:
        d = ' stroke-dasharray="5 3"' if lane == "ok" and not lb else (' stroke-dasharray="6 4"' if lane == "sum" else "")
        both = f' marker-start="url(#ah-l{lane})"' if lane == "sum" else ""
        svg.add(f'<path d="{arc_path(cx, cy, r_px, a0, a1)}" fill="none" stroke="{LANE[lane]}" stroke-width="2.6"{d} '
                f'marker-end="url(#ah-l{lane})"{both}/>')
    for r_px, a0, a1, lane, lb, at in paths:
        if lb:
            bx, by = polar(cx, cy, r_px, at)
            pill(svg, bx, by, lb, LANE[lane])
    # 순서 글
    tx, ty = x + 430, y + 66
    rows = [
        ("A", LANE["sum"], "대차 → ① 카세트 옮기기 (카세트마다 1 번)"),
        ("1–5", LANE["common"], f"① 슬롯 {n} 꺼냄 → ② 로 → 촬영 · 판정 → 다시 집기"),
        ("OK", LANE["ok"], f"6–11  UV {g(C.UV_S)} s → ① 원래 슬롯에 되돌림"),
        ("NG", LANE["ng"], f"6'–7'  ③ 위 슬롯부터 (지금 슬롯 {k})"),
        ("반복", INK2, f"슬롯 1 → {C.SLOT_COUNT} 아래부터, 빈 슬롯은 건너뜀"),
        ("B", LANE["sum"], f"{C.SLOT_COUNT} 장 끝 → ① 카세트를 대차로 → 다음 받침"),
    ]
    for tag_, col, txt in rows:
        svg.rect(tx, ty - 15, 50, 21, col, "none", rx=10)
        svg.text(tx + 25, ty, tag_, size=12, weight=700, fill="#ffffff", anchor="middle")
        svg.text(tx + 60, ty, txt, size=13, fill=INK2)
        ty += 36
    svg.add("</g>")


def pill(svg, x, y, label, color, size=11):
    w = max(18.0, est_w(label, size) + 10)
    svg.add('<g class="badge">')
    svg.rect(x - w / 2, y - 9, w, 18, color, "#ffffff", 1.2, rx=9)
    svg.text(x, y + 4, label, size=size, weight=700, fill="#ffffff", anchor="middle")
    svg.add("</g>")


def st_a(name):
    return C.STATIONS[name]


def draw_legend(svg, x, y, w, h):
    svg.add('<g class="node">')
    svg.rect(x, y, w, h, "#ffffff", "#94a3b8", 1.2, rx=8)
    svg.text(x + 16, y + 28, "읽는 법", size=16, weight=700)
    yy = y + 58
    for kind, lab in (("rot", "θ 선회"), ("z", "Z 오르내림"), ("r", "R 전진 · 후진"), ("capture", "촬영"), ("uv", "UV 조사")):
        svg.line(x + 18, yy - 4, x + 62, yy - 4, AX[kind], 2.6, arrow=kind if kind in ("z", "r", "rot") else None)
        svg.badge(x + 80, yy - 4, "1", AX[kind])
        svg.text(x + 98, yy, lab, size=13, fill=INK2)
        yy += 26
    svg.text(x + 18, yy + 2, "숫자 = 그 칸 안의 움직임 순서 (아래 글과 같음)", size=12, fill=INK3)
    yy += 26
    for kind, lab in (("u", "검사 전 웨이퍼"), ("ok", "정상(OK)"), ("ng", "불량(NG)")):
        f, s = WAFER[kind]
        svg.rect(x + 18, yy - 9, 44, 6, f, s, 1)
        svg.text(x + 72, yy - 2, lab, size=12.5, fill=INK2)
        yy += 20
    svg.rect(x + 18, yy - 11, 44, 10, NOGO[0], NOGO[1], 0.8, extra=' fill-opacity="0.55" stroke-dasharray="4 3"')
    svg.text(x + 72, yy - 2, "걸리면 안 되는 높이 (② 크래들 띠)", size=12, fill=INK2)
    yy += 20
    svg.rect(x + 18, yy - 11, 44, 10, OKZONE[0], OKZONE[1], 0.8, extra=' fill-opacity="0.7" stroke-dasharray="4 3"')
    svg.text(x + 72, yy - 2, "이 높이 안에서만 (UV 탑 밑)", size=12, fill=INK2)
    yy += 20
    svg.rect(x + 18, yy - 11, 44, 10, ROBOT[0], ROBOT[1], 0.8, extra=' opacity="0.3"')
    svg.text(x + 72, yy - 2, "옅은 로봇 = 칸 시작 자세", size=12, fill=INK2)
    svg.add("</g>")


def draw_rules(svg, x, y, w, h, n):
    lo, hi = C.CRADLE_BAND_Z
    tz = [C.transfer_z(i) for i in range(1, C.SLOT_COUNT + 1)]
    sz = [C.side_z(i) for i in range(1, C.SLOT_COUNT + 1)]
    low = [i + 1 for i in range(C.SLOT_COUNT) if tz[i] != sz[i]]
    low_t = ", ".join(f"슬롯 {i} 은 Z {g(sz[i - 1])} → {g(tz[i - 1])}" for i in low) or "내리는 슬롯 없음"
    svg.add('<g class="node">')
    svg.rect(x, y, w, h, "#ffffff", "#94a3b8", 1.2, rx=8)
    svg.text(x + 16, y + 28, "움직임 규칙 (코드가 강제 · sequence.can_rotate / uv_interlock)", size=16, weight=700)
    lines = [
        ("선회", f"기본은 Z {g(C.ZC_SWING)} · R 후진(R Sharp 로 RX ≤ {g(C.RX_RET_OK)})에서만 돈다."),
        ("높이 선회", f"① ↔ ② 는 웨이퍼를 든 채 들기 높이 그대로 돈다(사용자 변경). ② 크래들 띠 z {g(lo)}~{g(hi)} ± {g(C.HIGH_SWING_MARGIN)} 밖이어야 함 - {low_t}."),
        ("카세트", f"대차 ↔ ① 카세트 운반은 Z {g(C.CASS_Z_CARRY)} 에서 θ {g(C.STATIONS['input'])} ↔ {g(C.STATIONS['cart'])} 짧은 길. ② 보정 미세 회전(≤ {g(C.CORR_DTH)}°)은 Z {g(C.CRADLE_ZC_SEAT)}."),
        ("UV", f"탑 밑(RX > {g(C.RX_RET_OK)})은 ZC {g(C.UV_ZC_HAND)}~{g(C.UV_ZC_IN)} 만. 램프 = 7 조건({C.UV_TH_SENSOR} · θ{g(C.STATIONS['uv'])}±{g(C.UV_THETA_TOL)}° · RX ≤ {g(C.RX_RET_OK)} · ZC ≤ {g(C.UV_ZC_LAMP_MAX)} · 넘김 · OK · D9)."),
        ("센서", "S4(포크 웨이퍼): 집은 뒤 ON, 놓고 뺀 뒤 OFF · TOF-IN/NG: 카세트 있음 · θ 자리 센서: 도착마다 · Z/R Sharp: 높이 · 후진 확인."),
        ("멈춤", f"PC 하트비트 {g(C.LINK_LOSS_S)} s 끊김 → UV 끔 + 축 동작 끝낸 뒤 일시정지 · 판정 {g(C.JUDGE_TIMEOUT_S)} s 없음 → JUDGE_TIMEOUT · E-STOP(D9)."),
    ]
    yy = y + 64
    for tag_, txt in lines:
        svg.rect(x + 16, yy - 15, 76, 21, "#f1f5f9", "#cbd5e1", 1, rx=10)
        svg.text(x + 54, yy, tag_, size=12, weight=700, fill=INK2, anchor="middle")
        svg.text(x + 104, yy, txt, size=13, fill=INK2)
        yy += 38
    svg.add("</g>")


def draw_next(svg, x, y, n):
    svg.add('<g class="node">')
    svg.rect(x, y, FW, FH, "#ffffff", INK2, 1.6, rx=8)
    svg.text(x + 16, y + 30, "다음 웨이퍼", size=16, weight=700)
    e1 = S.empty_steps(n)[0]
    items, _, _ = walk(e1.actions, (C.STATIONS["input"], C.side_z(n), C.STATION_RX["input"]))
    lines = [
        (INK2, f"슬롯 n+1 로 → 칸 1 부터 반복 (아래 → 위)"),
        (INK3, f"OK 뒤: ① θ{g(C.STATIONS['input'])} 에서 바로 시작"),
        (INK3, f"NG 뒤: ③ θ{g(C.STATIONS['ng'])} → θ{g(C.STATIONS['input'])} 빈 포크 선회"),
        (INK2, "빈 슬롯 (P3 에서 S4 OFF):"),
        (INK3, f"E1  {' → '.join(i[4] for i in items)} → 다음 슬롯"),
        (INK2, "③ 가득 / 없음 → WAIT_NG"),
        (INK3, "TOF-NG 없음 → 있음 = 새 카세트 → 계속"),
        (INK2, f"{C.SLOT_COUNT} 장 끝 → B"),
    ]
    yy = y + 62
    for col, txt in lines:
        svg.text(x + 16, yy, txt, size=13, fill=col)
        yy += 30
    svg.add("</g>")


def draw_table(svg, x, y, w, h, n):
    svg.add('<g class="node">')
    svg.rect(x, y, w, h, "#ffffff", "#94a3b8", 1.2, rx=8)
    svg.text(x + 16, y + 30, "슬롯마다 바뀌는 높이 (Z = 캐리지 지령 mm)", size=16, weight=700)
    lo, hi = C.CRADLE_BAND_Z
    cols = [("슬롯", 50), ("바닥 Sn", 100), ("집기 Z", 100), ("들기 Z", 100), ("② 로 도는 Z", 130), ("② 띠 틈", 120),
            ("정상 1 장", 110)]
    cx0 = x + 20
    yy = y + 66
    xx = cx0
    for name, cw in cols:
        svg.text(xx, yy, name, size=12.5, weight=700, fill=INK2)
        xx += cw
    svg.line(cx0, yy + 8, x + w - 20, yy + 8, "#cbd5e1", 1)
    yy += 34
    for i in range(1, C.SLOT_COUNT + 1):
        tz = C.transfer_z(i)
        bot, top = tz + C.BLADE_PLANE_OFS, tz + C.BLADE_PLANE_OFS + C.WAFER_T
        gap = lo - top if top <= lo else bot - hi
        side = "밑" if top <= lo else "위"
        vals = [str(i), g(C.slot_z(i)), g(C.pick_z(i)), g(C.side_z(i)),
                g(tz) + (" (내림)" if tz != C.side_z(i) else ""), f"{side} {g(round(gap, 1))}", f"{wafer_time(i):.1f} s"]
        if i == n:
            svg.rect(cx0 - 8, yy - 19, w - 24, 28, "#eef2ff", "none", rx=4)
        xx = cx0
        for (name, cw), v in zip(cols, vals):
            hot = name == "② 로 도는 Z" and tz != C.side_z(i)
            svg.text(xx, yy, v, size=13, weight=700 if hot else 400, fill=AX["z"] if hot else INK2)
            xx += cw
        yy += 34
    svg.text(cx0, y + h - 14, f"틈 = 운반 웨이퍼와 ② 크래들 띠 {g(lo)}~{g(hi)} 사이 · 여유 {g(C.HIGH_SWING_MARGIN)} mm 이상(가정)",
             size=12, fill=INK3)
    svg.add("</g>")


def draw_arrows(svg, pos, nxt):
    def right(no_a, no_b, col):
        (xa, ya), (xb, yb) = pos[no_a], pos[no_b]
        svg.line(xa + FW + 2, ya + FH / 2, xb - 3, yb + FH / 2, col, 2.4, arrow="l" + lane_of(col))

    for a, b in (("1", "2"), ("2", "3"), ("3", "4"), ("4", "5"), ("6", "7"), ("7", "8"), ("8", "9"), ("9", "10"),
                 ("10", "11"), ("6'", "7'")):
        right(a, b, LANE[{"6'": "ng"}.get(a, "ok" if a in ("6", "7", "8", "9", "10") else "common")])
    # A -> 1
    xa, ya = COLX[0], ROW_Y[1]
    svg.line(xa + FW + 2, ya + FH / 2, COLX[1] - 3, ya + FH / 2, LANE["sum"], 2.4, arrow="lsum")
    # 5 -> 판정 마름모(왼쪽 여백) -> OK 6 / NG 6'
    x5, y5 = pos["5"]
    gy = ROW_Y[1] + FH + (ROW_Y[2] - ROW_Y[1] - FH) / 2
    dx, dy = 96, ROW_Y[2] + 78
    svg.poly([(x5 + FW / 2, y5 + FH + 2), (x5 + FW / 2, gy), (dx, gy), (dx, dy - 24)], LANE["common"], 2.4, arrow="lcommon")
    svg.add(f'<polygon points="{dx},{dy - 22} {dx + 26},{dy} {dx},{dy + 22} {dx - 26},{dy}" fill="#ffffff" '
            f'stroke="{LANE["common"]}" stroke-width="2"/>')
    svg.text(dx, dy + 4.5, "판정", size=12, weight=700, fill=LANE["common"], anchor="middle")
    x6, y6 = pos["6"]
    svg.line(dx + 27, dy, x6 - 3, dy, LANE["ok"], 2.4, arrow="lok")
    svg.text(dx + 30, dy - 8, "OK", size=12.5, weight=700, fill=LANE["ok"])
    xn, yn = pos["6'"]
    ny = yn + 78
    svg.poly([(dx, dy + 23), (dx, ny), (xn - 3, ny)], LANE["ng"], 2.4, arrow="lng")
    svg.text(dx + 6, ny - 8, "NG", size=12.5, weight=700, fill=LANE["ng"])
    # 11 -> 다음, 7' -> 다음, 다음 -> B
    nx, ny0 = nxt
    x11, y11 = pos["11"]
    gy2 = ROW_Y[2] + FH + (ROW_Y[3] - ROW_Y[2] - FH) / 2
    svg.poly([(x11 + FW / 2, y11 + FH + 2), (x11 + FW / 2, gy2), (nx + FW / 2, gy2), (nx + FW / 2, ny0 - 3)],
             LANE["ok"], 2.4, arrow="lok")
    x7, y7 = pos["7'"]
    svg.line(x7 + FW + 2, y7 + FH / 2, nx - 3, y7 + FH / 2, LANE["ng"], 2.4, arrow="lng")
    svg.line(nx + FW + 2, ny0 + FH / 2, COLX[3] - 3, ny0 + FH / 2, LANE["sum"], 2.4, arrow="lsum")


def lane_of(col):
    return {v: k for k, v in LANE.items()}[col]


def draw_lane_labels(svg):
    for row, lane, a, b in ((1, "common", "공통", ""), (2, "ok", "정상", "OK"), (3, "ng", "불량", "NG")):
        y = ROW_Y[row] + 190
        svg.rect(12, y - 22, 64, b and 50 or 30, LANE[lane], "none", rx=10)
        svg.text(44, y - 2, a, size=15, weight=700, fill="#ffffff", anchor="middle")
        if b:
            svg.text(44, y + 19, b, size=14, weight=700, fill="#ffffff", anchor="middle")


def draw_times(svg, fs, n, k):
    y = TIME_Y
    svg.text(30, y, f"시간 막대 (웨이퍼 1 장, ① 슬롯 {n}) — 칸 번호 = 위 칸", size=16, weight=700)
    ok = [fr for fr in fs if fr["lane"] in ("common", "ok")]
    ng = [fr for fr in fs if fr["lane"] in ("common", "ng")]
    t_ok, t_ng = sum(fr["t"] for fr in ok), sum(fr["t"] for fr in ng)
    x0, x1 = COLX[0], W - 150
    sc = (x1 - x0) / max(t_ok, t_ng)
    for i, (label, lane, seq, tot) in enumerate((("정상", "ok", ok, t_ok), ("불량", "ng", ng, t_ng))):
        by = y + 22 + i * 52
        svg.rect(12, by, 64, 32, LANE[lane], "none", rx=10)
        svg.text(44, by + 21, label, size=14, weight=700, fill="#ffffff", anchor="middle")
        xx = x0
        for j, fr in enumerate(seq):
            ww = fr["t"] * sc
            op = "0.95" if j % 2 == 0 else "0.7"
            svg.rect(xx, by, ww, 32, LANE[fr["lane"]], "#ffffff", 1.5, extra=f' fill-opacity="{op}"')
            if ww >= 26:
                svg.text(xx + ww / 2, by + 21, fr["no"], size=12.5, weight=700, fill="#ffffff", anchor="middle")
            xx += ww
        svg.text(xx + 10, by + 22, f"{tot:.1f} s", size=15, weight=700, fill=LANE[lane])
    yy = y + 22 + 2 * 52 + 14
    oks = " · ".join(f"{i}: {wafer_time(i):.1f}" for i in range(1, C.SLOT_COUNT + 1))
    ngs = " · ".join(f"{i}: {wafer_time(i, ok=False, k=k):.1f}" for i in range(1, C.SLOT_COUNT + 1))
    svg.text(COLX[0], yy, f"슬롯별 1 장 시간 (s) — 정상  {oks}   |   불량(③ 슬롯 {k})  {ngs}   |   "
                      f"대차 → ① 반입 · ① → 대차 반출은 카세트마다 따로 (A · B)", size=13, fill=INK2)


def main(argv=None):
    ap = argparse.ArgumentParser(description="R22 process flow storyboard SVG")
    ap.add_argument("--slot", type=int, default=1)
    ap.add_argument("--ng-slot", type=int, default=C.SLOT_COUNT)
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    a = ap.parse_args(argv)
    out = pathlib.Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    s = build_svg(a.slot, a.ng_slot)
    out.write_text(s, encoding="utf-8", newline="\n")
    print(f"wrote {out} ({len(s)} chars, slot {a.slot}, ng slot {a.ng_slot})")


if __name__ == "__main__":
    main()
