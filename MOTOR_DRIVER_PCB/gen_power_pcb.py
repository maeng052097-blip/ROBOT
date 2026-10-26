# -*- coding: utf-8 -*-
"""
전원분배 PCB-A / PCB-B 스키매틱 생성기 (KiCad 10)

설계근거: MOTOR_DRIVER_PCB/전원PCB_설계기준_24V2팩.md
 - PCB-A = 팩A(바퀴): MDD10A x2, 12V벅(팬x2 + 릴레이코일), ULN2003, 볼트미터, 전압감시
 - PCB-B = 팩B(팔)  : 스테퍼 x3(각 5A퓨즈), XL4016 x2, 볼트미터, 전압감시

방식: 심볼을 KiCad 표준 라이브러리에서 읽어 lib_symbols 에 삽입 -> 인스턴스 배치
      -> 각 핀에서 스터브 배선 + 네트라벨. 라벨 이름이 같으면 같은 네트.
"""
import os, re, io, uuid as U

KICAD  = r"C:\Users\MSY\AppData\Local\Programs\KiCad\10.0\share\kicad"
SYMDIR = os.path.join(KICAD, "symbols")
FPDIR  = os.path.join(KICAD, "footprints")
OUTROOT = r"C:\Users\MSY\Desktop\ROBOT\MOTOR_DRIVER_PCB"

STUB = 10.16          # 핀에서 라벨까지 스터브 길이(그리드 배수)
GRID = 1.27


# ---------------------------------------------------------------- S식 파서
def find_blocks(text, opener):
    """'(opener ' 로 시작하는 최상위 괄호블록들을 (start, end, body) 로 반환."""
    out, i = [], 0
    tok = "(" + opener
    while True:
        j = text.find(tok, i)
        if j < 0:
            break
        # opener 뒤가 공백/개행이어야 함 (pin vs pin_names 구분)
        if text[j + len(tok)] not in " \t\n":
            i = j + 1
            continue
        d, k = 0, j
        while True:
            c = text[k]
            if c == '"':                       # 문자열 건너뛰기
                k += 1
                while text[k] != '"' or text[k - 1] == "\\":
                    k += 1
            elif c == "(":
                d += 1
            elif c == ")":
                d -= 1
                if d == 0:
                    break
            k += 1
        out.append((j, k + 1, text[j:k + 1]))
        i = k + 1
    return out


_LIBCACHE = {}


def load_symbol(lib, name):
    """KiCad 심볼 라이브러리에서 심볼 1개를 꺼내 (블록텍스트, 핀목록) 반환.
    블록의 최상위 이름을 'lib:name' 으로 바꾼다(스키매틱 lib_symbols 규약)."""
    if lib not in _LIBCACHE:
        with io.open(os.path.join(SYMDIR, lib + ".kicad_sym"), encoding="utf-8") as f:
            _LIBCACHE[lib] = f.read()
    text = _LIBCACHE[lib]

    target = '(symbol "%s"' % name
    blk = None
    for s, e, b in find_blocks(text, "symbol"):
        if b.startswith(target):
            blk = b
            break
    if blk is None:
        raise KeyError("심볼 없음: %s:%s" % (lib, name))

    # 핀 추출: (pin TYPE STYLE (at x y a) (length L) ... (number "N" ...))
    pins = []
    for _, _, pb in find_blocks(blk, "pin"):
        m_at = re.search(r"\(at\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\)", pb)
        m_no = re.search(r'\(number\s+"([^"]*)"', pb)
        if m_at and m_no:
            pins.append({
                "num": m_no.group(1),
                "x": float(m_at.group(1)),
                "y": float(m_at.group(2)),
                "a": int(float(m_at.group(3))),
            })

    # 최상위 이름만 'lib:name' 으로 교체 (중첩 sub-symbol 이름은 그대로 둔다)
    blk = blk.replace(target, '(symbol "%s:%s"' % (lib, name), 1)
    return blk, pins


# ---------------------------------------------------------------- 부품 정의
# (ref, lib, sym, value, footprint, x, y)  좌표 = 스키매틱 시트 mm
def A_PARTS():
    return [
      # --- 입력부 ---
      ("J1", "Connector_Generic", "Conn_01x02", "BAT_A_IN_XT60",
       "Connector_AMASS:AMASS_XT60-M_1x02_P7.20mm_Vertical",                         55, 45),
      ("F1", "Device", "Fuse", "30A_ATO",
       "Fuse:Fuseholder_Blade_ATO_Littelfuse_Pudenz_2_Pin", 55, 88),
      ("J2", "Connector_Generic", "Conn_01x02", "SW_A_40A",
       "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-3-2-5.08_1x02_P5.08mm_Horizontal", 55, 128),

      # --- 모터드라이버 출력 (최대전류) ---
      ("J3", "Connector_Generic", "Conn_01x02", "MD1_PWR_20A",
       "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-3-2-5.08_1x02_P5.08mm_Horizontal", 150, 45),
      ("J4", "Connector_Generic", "Conn_01x02", "MD2_PWR_20A",
       "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-3-2-5.08_1x02_P5.08mm_Horizontal", 150, 85),

      # --- 벌크캡 ---
      ("C1", "Device", "C_Polarized", "2200uF/50V",
       "Capacitor_THT:CP_Radial_D16.0mm_P7.50mm",     150, 130),
      ("C2", "Device", "C_Polarized", "2200uF/50V",
       "Capacitor_THT:CP_Radial_D16.0mm_P7.50mm",     150, 175),

      # --- 12V 계통 ---
      ("J5", "Connector_Generic", "Conn_01x04", "BUCK_24to12V_2A",
       "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical", 245, 45),
      ("J6", "Connector_Generic", "Conn_01x02", "FAN1_12V",
       "Connector_Molex:Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical",  245, 95),
      ("J7", "Connector_Generic", "Conn_01x02", "FAN2_12V",
       "Connector_Molex:Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical",  245, 135),

      # --- 타워램프 (외부 릴레이 모듈 구동) ---
      ("U1", "Transistor_Array", "ULN2003", "ULN2003A",
       "Package_DIP:DIP-16_W7.62mm_Socket",           340, 70),
      ("J8", "Connector_Generic", "Conn_01x04", "RELAY_MOD_PWR",
       "Connector_Molex:Molex_SPOX_5267-04A_1x04_P2.50mm_Vertical",  245, 180),
      ("J9", "Connector_Generic", "Conn_01x05", "RELAY_MOD_SIG",
       "Connector_Molex:Molex_SPOX_5267-05A_1x05_P2.50mm_Vertical",  405, 70),

      # --- 계측 / 쉴드 ---
      ("R1", "Device", "R", "100k",
       "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", 55, 175),
      ("R2", "Device", "R", "18k",
       "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", 55, 220),
      ("J10", "Connector_Generic", "Conn_01x02", "VOLTMETER_A",
       "Connector_Molex:Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical",  150, 220),
      ("J11", "Connector_Generic", "Conn_01x08", "TO_SHIELD",
       "Connector_Molex:Molex_SPOX_5267-08A_1x08_P2.50mm_Vertical",  340, 185),
    ]


def A_NETS():
    return {
      # 배터리 -> 퓨즈 -> 스위치(외부 왕복) -> 24V 버스
      "BAT_A+":  [("J1", "1"), ("F1", "1")],
      "F1_OUT":  [("F1", "2"), ("J2", "1")],
      "+24V": [("J2", "2"), ("J3", "1"), ("J4", "1"), ("J5", "1"),
               ("J8", "1"), ("J10", "1"), ("C1", "1"), ("C2", "1"), ("R1", "1")],
      "+12V": [("J5", "3"), ("J6", "1"), ("J7", "1"), ("J8", "2")],
      "GND":  [("J1", "2"), ("J3", "2"), ("J4", "2"), ("J5", "2"), ("J5", "4"),
               ("J6", "2"), ("J7", "2"), ("J8", "3"), ("J8", "4"), ("J9", "5"),
               ("J10", "2"), ("J11", "6"), ("J11", "7"),
               ("U1", "8"), ("U1", "5"), ("U1", "6"), ("U1", "7"),
               ("C1", "2"), ("C2", "2"), ("R2", "2")],
      # 쉴드 신호 -> ULN2003 입력
      "LAMP1_IN": [("J11", "1"), ("U1", "1")],
      "LAMP2_IN": [("J11", "2"), ("U1", "2")],
      "LAMP3_IN": [("J11", "3"), ("U1", "3")],
      "LAMP4_IN": [("J11", "4"), ("U1", "4")],
      # ULN2003 오픈컬렉터 출력 -> 릴레이 모듈 IN
      "LAMP1_OUT": [("U1", "16"), ("J9", "1")],
      "LAMP2_OUT": [("U1", "15"), ("J9", "2")],
      "LAMP3_OUT": [("U1", "14"), ("J9", "3")],
      "LAMP4_OUT": [("U1", "13"), ("J9", "4")],
      # 배터리 전압 감시 분압 -> 쉴드 아날로그핀
      "VSENSE_A": [("R1", "2"), ("R2", "1"), ("J11", "5")],
    }


# U1 COM(9), 미사용 출력(10,11,12), J11.8 은 의도적 미연결 -> no_connect
A_NC = [("U1", "9"), ("U1", "10"), ("U1", "11"), ("U1", "12"), ("J11", "8")]
A_FLAGS = ["+24V", "+12V", "GND"]


def B_PARTS():
    return [
      ("J1", "Connector_Generic", "Conn_01x02", "BAT_B_IN_XT60",
       "Connector_AMASS:AMASS_XT60-M_1x02_P7.20mm_Vertical",                         55, 45),
      ("F1", "Device", "Fuse", "10A_ATO",
       "Fuse:Fuseholder_Blade_ATO_Littelfuse_Pudenz_2_Pin", 55, 88),
      ("J2", "Connector_Generic", "Conn_01x02", "SW_B_15A",
       "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-3-2-5.08_1x02_P5.08mm_Horizontal", 55, 128),
      ("C1", "Device", "C_Polarized", "1000uF/50V",
       "Capacitor_THT:CP_Radial_D12.5mm_P5.00mm",     55, 175),

      # 스테퍼 3개 : 각각 5A 고속퓨즈 (드라이버 매뉴얼 7.1절 권장)
      ("F2", "Device", "Fuse", "5A_FAST",
       "Fuse:Fuseholder_Clip-5x20mm_Keystone_3512_Inline_P23.62x7.27mm_D1.02x1.57mm_Horizontal", 150, 45),
      ("J3", "Connector_Generic", "Conn_01x02", "ST1_PWR",
       "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-3-2-5.08_1x02_P5.08mm_Horizontal", 225, 45),
      ("F3", "Device", "Fuse", "5A_FAST",
       "Fuse:Fuseholder_Clip-5x20mm_Keystone_3512_Inline_P23.62x7.27mm_D1.02x1.57mm_Horizontal", 150, 95),
      ("J4", "Connector_Generic", "Conn_01x02", "ST2_PWR",
       "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-3-2-5.08_1x02_P5.08mm_Horizontal", 225, 95),
      ("F4", "Device", "Fuse", "5A_FAST",
       "Fuse:Fuseholder_Clip-5x20mm_Keystone_3512_Inline_P23.62x7.27mm_D1.02x1.57mm_Horizontal", 150, 145),
      ("J5", "Connector_Generic", "Conn_01x02", "ST3_PWR",
       "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-3-2-5.08_1x02_P5.08mm_Horizontal", 225, 145),

      # 서보 벅 2개 (XL4016 모듈은 보드 밖. 4핀으로 입력/출력을 모두 보드에 물려
      #              벅 출력 GND 가 PCB-B 단일 GND 폴리곤에 붙게 한다)
      #   1=IN+  2=IN-  3=OUT+  4=OUT-
      ("J6", "Connector_Generic", "Conn_01x04", "XL4016_1_7V4",
       "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-3-4-5.08_1x04_P5.08mm_Horizontal", 225, 195),
      ("J7", "Connector_Generic", "Conn_01x04", "XL4016_2_12V",
       "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-3-4-5.08_1x04_P5.08mm_Horizontal", 225, 245),

      # 서보 전원 출력 (TTL 은 쉴드 HK1 에서 따로 간다. 여기는 전원만)
      ("J10", "Connector_Generic", "Conn_01x02", "DRS0201_7V4",
       "Connector_Molex:Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical",  150, 200),
      ("J11", "Connector_Generic", "Conn_01x02", "DRS0601_12V",
       "Connector_Molex:Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical",  150, 245),

      # 계측 / 쉴드
      ("R1", "Device", "R", "100k",
       "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", 335, 50),
      ("R2", "Device", "R", "18k",
       "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", 335, 95),
      ("J8", "Connector_Generic", "Conn_01x02", "VOLTMETER_B",
       "Connector_Molex:Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical",  335, 150),
      ("J9", "Connector_Generic", "Conn_01x02", "TO_SHIELD_SENSE",
       "Connector_Molex:Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical",  335, 195),
    ]


def B_NETS():
    return {
      "BAT_B+": [("J1", "1"), ("F1", "1")],
      "F1_OUT": [("F1", "2"), ("J2", "1")],
      "+24V_B": [("J2", "2"), ("F2", "1"), ("F3", "1"), ("F4", "1"),
                 ("J6", "1"), ("J7", "1"), ("J8", "1"),
                 ("C1", "1"), ("R1", "1")],
      # 벅 출력 -> 서보 (XL4016 은 비절연이라 OUT- = IN- = GND)
      "SERVO_7V4": [("J6", "3"), ("J10", "1")],
      "SERVO_12V": [("J7", "3"), ("J11", "1")],
      "ST1_V+": [("F2", "2"), ("J3", "1")],
      "ST2_V+": [("F3", "2"), ("J4", "1")],
      "ST3_V+": [("F4", "2"), ("J5", "1")],
      "GND":    [("J1", "2"), ("J3", "2"), ("J4", "2"), ("J5", "2"),
                 ("J6", "2"), ("J6", "4"), ("J7", "2"), ("J7", "4"),
                 ("J8", "2"), ("J9", "2"), ("J10", "2"), ("J11", "2"),
                 ("C1", "2"), ("R2", "2")],
      "VSENSE_B": [("R1", "2"), ("R2", "1"), ("J9", "1")],
    }


B_NC = []
B_FLAGS = ["+24V_B", "GND"]


# ---------------------------------------------------------------- 생성기
def uid():
    return str(U.uuid4())


def snap(v):
    return round(round(v / GRID) * GRID, 4)


def gen(project, parts, nets, ncs, flags, outdir, title):
    os.makedirs(outdir, exist_ok=True)
    root = uid()

    # 배치 원점을 1.27mm 그리드에 스냅한다.
    # 원점이 그리드를 벗어나면 핀 좌표도 벗어나 배선이 핀에 안 닿는다(ERC 대량 오류).
    parts = [(r, l, sy, v, f, snap(x), snap(y)) for (r, l, sy, v, f, x, y) in parts]

    # PWR_FLAG 를 일반 부품처럼 편입 (ERC 'power input not driven' 방지)
    nets = {k: list(v) for k, v in nets.items()}
    for i, net in enumerate(flags):
        ref = "#FLG%d" % (i + 1)
        parts.append((ref, "power", "PWR_FLAG", "PWR_FLAG", "",
                      snap(55 + i * 45), snap(262)))
        nets[net] = nets[net] + [(ref, "1")]

    # 1) 심볼 로드
    syms = {}
    for ref, lib, sym, val, fp, x, y in parts:
        key = "%s:%s" % (lib, sym)
        if key not in syms:
            syms[key] = load_symbol(lib, sym)

    # 2) 핀 -> 네트 역인덱스
    pin2net = {}
    for net, members in nets.items():
        for ref, num in members:
            if (ref, num) in pin2net:
                raise ValueError("핀 중복 배정: %s.%s (%s, %s)"
                                 % (ref, num, pin2net[(ref, num)], net))
            pin2net[(ref, num)] = net

    L = []
    L.append("(kicad_sch")
    L.append('\t(version 20260306)')
    L.append('\t(generator "gen_power_pcb.py")')
    L.append('\t(generator_version "10.0")')
    L.append('\t(uuid "%s")' % root)
    L.append('\t(paper "A3")')
    L.append('\t(title_block')
    L.append('\t\t(title "%s")' % title)
    L.append('\t\t(date "2026-09-03")')
    L.append('\t\t(comment 1 "Ref: MOTOR_DRIVER_PCB/power PCB design spec, 24V 2-pack")')
    L.append('\t)')

    # 3) lib_symbols
    L.append("\t(lib_symbols")
    for key in sorted(syms):
        for line in syms[key][0].splitlines():
            L.append("\t\t" + line.lstrip("\t") if line.strip() else line)
    L.append("\t)")

    wires, labels, ncmarks = [], [], []

    # 4) 심볼 인스턴스 + 스터브 + 라벨
    for ref, lib, sym, val, fp, x, y in parts:
        key = "%s:%s" % (lib, sym)
        pins = syms[key][1]
        L.append("\t(symbol")
        L.append('\t\t(lib_id "%s")' % key)
        L.append("\t\t(at %s %s 0)" % (x, y))
        L.append("\t\t(unit 1)")
        L.append("\t\t(body_style 1)")
        L.append("\t\t(exclude_from_sim no)(in_bom yes)(on_board yes)(in_pos_files yes)(dnp no)")
        L.append('\t\t(uuid "%s")' % uid())
        # 핀이 위/아래로 나오는 부품(저항/캡/퓨즈)은 참조/값을 옆으로 빼야
        # 네트라벨과 겹치지 않는다.
        vertical = pins and all((pp["a"] % 360) in (90, 270) for pp in pins)
        if vertical:
            props = (("Reference", ref, 8.89, -1.27), ("Value", val, 8.89, 3.81),
                     ("Footprint", fp, 8.89, 6.35), ("Datasheet", "", 8.89, 8.89))
        else:
            props = (("Reference", ref, 0, -12.7), ("Value", val, 0, 12.7),
                     ("Footprint", fp, 0, 15.24), ("Datasheet", "", 0, 17.78))
        for pname, pval, dxp, dy in props:
            hide = "\n\t\t\t(hide yes)" if (pname in ("Footprint", "Datasheet")
                                               or ref.startswith("#")) else ""
            L.append('\t\t(property "%s" "%s"' % (pname, pval))
            L.append("\t\t\t(at %s %s 0)%s" % (snap(x + dxp), snap(y + dy), hide))
            L.append("\t\t\t(effects (font (size 1.27 1.27)))")
            L.append("\t\t)")
        for p in pins:
            L.append('\t\t(pin "%s" (uuid "%s"))' % (p["num"], uid()))
        L.append("\t\t(instances")
        L.append('\t\t\t(project "%s"' % project)
        L.append('\t\t\t\t(path "/%s" (reference "%s") (unit 1))' % (root, ref))
        L.append("\t\t\t)")
        L.append("\t\t)")
        L.append("\t)")

        # 스터브 + 라벨 (핀 절대좌표: 시트 Y 는 심볼 Y 의 반대부호)
        for p in pins:
            px, py = round(x + p["x"], 4), round(y - p["y"], 4)
            a = p["a"] % 360
            dx, dy_, lang = {0: (-1, 0, 180), 180: (1, 0, 0),
                             90: (0, 1, 270), 270: (0, -1, 90)}[a]
            ex, ey = round(px + dx * STUB, 4), round(py + dy_ * STUB, 4)
            net = pin2net.get((ref, p["num"]))
            if net is None:
                if (ref, p["num"]) in ncs:
                    ncmarks.append((px, py))
                else:
                    raise ValueError("네트 미배정 & NC 미선언: %s.%s" % (ref, p["num"]))
                continue
            wires.append((px, py, ex, ey))
            labels.append((net, ex, ey, lang))

    # 6) 배선 / 라벨 / no_connect
    for x1, y1, x2, y2 in wires:
        L.append("\t(wire")
        L.append("\t\t(pts (xy %s %s) (xy %s %s))" % (x1, y1, x2, y2))
        L.append("\t\t(stroke (width 0) (type default))")
        L.append('\t\t(uuid "%s")' % uid())
        L.append("\t)")
    for net, x, y, ang in labels:
        L.append('\t(label "%s"' % net)
        L.append("\t\t(at %s %s %s)" % (x, y, ang))
        L.append("\t\t(effects (font (size 1.27 1.27)) (justify left bottom))")
        L.append('\t\t(uuid "%s")' % uid())
        L.append("\t)")
    for x, y in ncmarks:
        L.append('\t(no_connect (at %s %s) (uuid "%s"))' % (x, y, uid()))

    L.append("\t(sheet_instances")
    L.append('\t\t(path "/" (page "1"))')
    L.append("\t)")
    L.append("\t(embedded_fonts no)")
    L.append(")")

    schpath = os.path.join(outdir, project + ".kicad_sch")
    with io.open(schpath, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")

    pro = ('{\n  "board": {"design_settings": {}},\n'
           '  "meta": {"filename": "%s.kicad_pro", "version": 3},\n'
           '  "schematic": {"legacy_lib_dir": "", "legacy_lib_list": []},\n'
           '  "sheets": [["%s", "Root"]],\n  "text_variables": {}\n}\n' % (project, root))
    with io.open(os.path.join(outdir, project + ".kicad_pro"), "w", encoding="utf-8") as f:
        f.write(pro)

    return schpath, len(parts), len(nets), len(wires)


# ---------------------------------------------------------------- 풋프린트 존재 확인
def check_fps(parts):
    miss = []
    for p in parts:
        fp = p[4]
        if not fp:
            continue
        lib, name = fp.split(":", 1)
        path = os.path.join(FPDIR, lib + ".pretty", name + ".kicad_mod")
        if not os.path.exists(path):
            miss.append((p[0], fp))
    return miss


if __name__ == "__main__":
    import sys
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    jobs = [
        ("PCB_POWER_A", A_PARTS(), A_NETS(), A_NC, A_FLAGS,
         "PCB-A  Power Distribution  (Pack A / Wheels)"),
        ("PCB_POWER_B", B_PARTS(), B_NETS(), B_NC, B_FLAGS,
         "PCB-B  Power Distribution  (Pack B / Arm)"),
    ]
    for proj, parts, nets, ncs, flags, title in jobs:
        outdir = os.path.join(OUTROOT, proj)
        path, np_, nn, nw = gen(proj, parts, nets, ncs, flags, outdir, title)
        print("[%s] 부품 %d개 / 네트 %d개 / 배선 %d개" % (proj, np_, nn, nw))
        print("   ->", path)
        miss = check_fps(parts)
        if miss:
            print("   [풋프린트 없음]")
            for ref, fp in miss:
                print("      %-5s %s" % (ref, fp))
        else:
            print("   풋프린트 전부 확인됨")
