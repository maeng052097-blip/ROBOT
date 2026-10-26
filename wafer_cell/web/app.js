/* R22 웨이퍼 검사셀 공정 모니터 — 브라우저 쪽.
   서버(SSE 20Hz) 스냅샷을 받아 벽시계 기준으로 보간해 도면을 움직인다. SIM(시뮬)과 LIVE(셀 Mega)가 같은 스냅샷 모양.
   평면도: 셀좌표(y 아래로 증가). θ 0°=+x(③ 불량), 90°=위. 로봇 그룹 rotate(-θ).
   스테이션 = ③ 불량 0 · UV 60(사용자 변경) · ② 검사 120 · ① 투입 180 (22 mm 바깥) · 대차 270. ① <-> ② 는 높이 선회(사용자 변경, 옆길 없음).
   입면도 = 포크 방향 r-z 단면(포크는 항상 오른쪽), y = -z.
*/
'use strict';
const BOOT = window.BOOT || { token: null, view_only: true };
const $ = (id) => document.getElementById(id);
const NS = 'http://www.w3.org/2000/svg';
function el(tag, attrs, parent) {
  const e = document.createElementNS(NS, tag);
  for (const k in (attrs || {})) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
}
function tx(s, attrs, parent) { const e = el('text', attrs, parent); e.textContent = s; return e; }
function rect(x, y, w, h, cls, parent, extra) { return el('rect', Object.assign({ x, y, width: w, height: h, class: cls }, extra || {}), parent); }
function circ(cx, cy, r, cls, parent) { return el('circle', { cx, cy, r, class: cls }, parent); }
const fmt = (v, d = 1) => (v === null || v === undefined || Number.isNaN(v)) ? '-' : Number(v).toFixed(d);
function store(k, v) { try { if (v === undefined) return localStorage.getItem(k); localStorage.setItem(k, v); } catch (e) { return null; } }
const show = (e, on) => { e.style.display = on ? '' : 'none'; };
function esc(s) { return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])); }

const STATE_KR = {
  IDLE: '원점 필요', HOMING: '원점 복귀 중', READY: '대기', RUNNING: '공정 진행',
  WAIT_CART: '대차 대기', WAIT_NG: '불량 카세트 교체', PAUSED: '일시정지', ALARM: '알람', ESTOP: '비상정지',
};
const LINK_KR = { OK: '장비 연결', DOWN: '연결 끊김', WAIT_HELLO: '응답 대기', STALE: '데이터 멈춤', VERSION: '프로토콜 불일치', NO_PYSERIAL: 'pyserial 없음', SIM: '' };
const FAULT_KR = {
  th_double: 'θ 센서 2 개 LOW (θ0 붙음)', th_dead: 'θ180 센서 죽음', belt_skip: '벨트 이빨 튐 (θ 3°)',
  z_sharp_drift: 'Z Sharp +6 mm 틀어짐', r_sharp_fail: 'R Sharp 고장 (160 고정)', s4_false_none: 'S4 거짓 없음 (1 회)',
  s4_false_present: 'S4 거짓 있음 (1 회)', tof_in_none: 'TOF-IN 못 봄', uv_led_fail: 'UV LED 안 켜짐 (A_UV)',
  uv_overtemp: 'UV 과열 (3 s 뒤 KSD9700)', corr_out: '치우침 보정 창 밖 (δr -5)', judge_drop: 'PC 판정 안 보냄',
  camera_fail: '카메라 실패 (ERR)', cart_delay: '대차 호출 느림', sim_exception: '시뮬 내부 예외',
};
const RUNNING_STATES = ['HOMING', 'RUNNING', 'WAIT_CART', 'WAIT_NG'];
const ALLOWED = {
  home: ['IDLE', 'READY'], start: ['READY'], pause: RUNNING_STATES, resume: ['PAUSED'],
  stop: [...RUNNING_STATES, 'PAUSED'], reset: ['ALARM', 'ESTOP'],
};
const CMD_KR = { home: '원점 복귀', start: '공정 시작', resume: '재개', reset: '알람 해제', pause: '일시정지', stop: '소프트 정지' };
const SIM_CONFIRM = ['home', 'start', 'resume', 'reset'];
const ST_KR = { input: '투입', inspect: '검사', ng: '불량', uv: 'UV', cart: '대차' };
const CODE_KR = { U: '있다고 봄 (미처리)', O: '꺼냄 (공정 중)', K: '정상 - 원래 자리', N: '불량 - ③ 으로', E: '빈 슬롯 (S4 없음)', '-': '카세트 없음' };
const CODE_CLS = { U: 'est', O: 'out', K: 'ok', N: 'ng', E: 'empty_conf', '-': 'none' };
const SEAT_KR = { F: '처리 전', W: '셀에서 처리 중', D: '끝' };
const SENSOR_OF_ALARM = { TH_MISMATCH: 'TH', TH_SENSOR_FAULT: 'TH', HOME_NO_STATION: 'TH', S4_REMAINS: 'S4', S4_MISSING: 'S4' };

let META = null, GEO = null;
const CAT = {};
let snap = null, bootId = null;
const buf = [];
let offset = null, lastMsgAt = 0, lastTick = -1, lastTickAt = 0, stale = false;
let camBusy = false, camSeq = 0;
let modalOpen = false, attnKey = null;
const R = {};            // SVG 참조

/* ───────────────────────── 평면도 ───────────────────────── */
const pos = (deg, r) => [r * Math.cos(deg * Math.PI / 180), -r * Math.sin(deg * Math.PI / 180)];
// 평면도 보기 회전: 대차(270°, 셀 앞)를 화면 오른쪽에 둔다(넓은 패널에 대차 받침 4칸이 다 들어가게).
// 화면에서 ③ 0° = 위, UV 60° = 왼쪽 위, ② 120° = 왼쪽 아래, ① 180° = 아래. 글자는 똑바로.
const VIEW = -90;
const posV = (deg, r) => pos(deg - VIEW, r);
let SR = 235.5, IR = 257.5, UR = 269.5;

function drawCassette(parent, kind, rc) {
  // 반경 방향 틀(입구가 로봇 쪽). R9 카세트 외형을 중심 rc 로 옮김.
  const g = el('g', { transform: `translate(${rc - 235.5},0)` }, parent);
  rect(250, -62, 50, 124, 'part2', g);
  rect(215, -62, 79, 5, 'part', g); rect(215, 57, 79, 5, 'part', g);
  rect(215, -57, 71, 10, 'part', g); rect(215, 47, 71, 10, 'part', g); rect(288, -57, 6, 114, 'part', g);
  rect(212, -65, 92, 130, 'cass-frame ' + kind, g);
  const w = circ(235.5, 0, 50, 'wafer ' + (kind === 'ng' ? 'ngw' : ''), g);
  return { g, w };
}

function buildPlan() {
  const svg = $('plan');
  svg.innerHTML = '';
  const g = el('g', { transform: `rotate(${VIEW})` }, svg);
  // 책상(공정은 책상 위, 대차는 책상 옆 바닥) - 치수는 가정(cell_config DESK_*)
  const dk = GEO.desk;
  R.desk = rect(dk.x0, -dk.y1, dk.x1 - dk.x0, dk.y1 - dk.y0, 'desk', g);
  const arc = (a0, a1, r, cls) => { const [x0, y0] = pos(a0, r), [x1, y1] = pos(a1, r); el('path', { d: `M${x0} ${y0} A${r} ${r} 0 ${Math.abs(a1 - a0) > 180 ? 1 : 0} 0 ${x1} ${y1}`, class: cls }, g); };
  arc(0, 180, SR, 'path');                                   // 웨이퍼 경로 0 -> 180
  // 높이 선회 ② <-> ① (웨이퍼를 든 채 슬롯 높이 그대로, 후진 운반 웨이퍼 중심). arc 는 a0 < a1 로 불러야 짧은 호(반시계)
  arc(GEO.stations.inspect, GEO.stations.input, 184.5, 'path high');
  arc(GEO.stations.input, GEO.stations.cart, 184.5, 'path carry');   // 카세트 운반 180 -> 270 짧은 길
  R.hi = {};
  for (const [k, a] of Object.entries(GEO.stations)) { const [x, y] = pos(a, GEO.station_r[k]); R.hi[k] = circ(x, y, 70, 'stn-hi', g); }
  // ① 투입 (22 mm 바깥, 클램프 윗판) · ③ 불량
  R.cass = {};
  for (const k of GEO.cassettes) {
    const sg = el('g', { transform: `rotate(${-GEO.stations[k]})` }, g);
    rect(GEO.station_r[k] - 23.5, -65, 92, 130, 'dock', sg);                   // 도크 자리(카세트가 없어도 보이게)
    R.cass[k] = drawCassette(sg, k, GEO.station_r[k]);
    if (k === 'input') rect(235.5 - 29.5, -70, 100, 140, 'clamp', R.cass[k].g);   // 카세트 그룹 좌표(웨이퍼 중심 235.5) - 3D 와 같은 자리
  }
  // ② 검사 크래들 (백라이트 C-03N 위 크래들 C-02N, 위에 StreamCam)
  const ig = el('g', { transform: `rotate(${-GEO.stations.inspect})` }, g);
  rect(262, -52, 56, 104, 'hub', ig); rect(220, -52, 45, 12, 'part', ig); rect(220, 40, 45, 12, 'part', ig);
  rect(238, -50, 30, 100, 'backlight', ig);
  for (const [x, y] of [[222, -43], [222, 43], [280.5, 0]]) circ(x, y, 4, 'pad', ig);
  R.inspW = circ(SR, 0, 50, 'wafer', ig);
  // UV 탑 (θ60): 탑 몸통 + 받침 패드 + 받침 웨이퍼 + 램프 빛
  const ug = el('g', { transform: `rotate(${-GEO.stations.uv})` }, g);
  R.uvGlow = circ(UR, 0, 74, 'uvglow', ug);
  circ(UR, 0, 62, 'uvtower', ug);
  for (const [x, y] of [[UR - 13.5, -43], [UR - 13.5, 43], [UR + 45, 0]]) circ(x, y, 4, 'pad', ug);
  R.uvW = circ(UR, 0, 50, 'wafer okw', ug);
  // 고정부: 페데스탈, θ 모터, θ 자리 센서 5 (TS-11 링)
  el('path', { d: 'M82.20,-21.65 A85,85 0 1,0 82.20,21.65 L33.5,21.65 L33.5,-21.65 Z', class: 'part2' }, g);
  rect(37.47, -21.15, 42.3, 42.3, 'part', g);
  // 대차(메카넘, 받침 2x2). 로컬: FL(0,0) FR(200,0) RL(200,260) RR(0,260), 중심(100,130), +y = 셀에서 멀어짐
  R.cartGhost = el('g', { transform: `translate(0,${UR})` }, g);
  const [bx, by, bw, bh] = GEO.cart_body;
  rect(bx, by, bw, bh, 'seat ghostcart', R.cartGhost);
  const cart = el('g', {}, g); R.cart = cart;
  rect(bx, by, bw, bh, 'cartbody', cart);
  for (const [x, y] of [[bx, by], [bx + bw - 40, by], [bx, by + bh - 40], [bx + bw - 40, by + bh - 40]]) rect(x, y, 40, 40, 'wheel', cart);
  R.seat = {};
  for (const [s, [x, y]] of Object.entries(GEO.cart_seat_xy)) {
    const sr = rect(x - 62, y - 62, 124, 124, 'seat', cart);
    // 카세트 입구가 그리퍼 쪽: 앞 받침은 셀 쪽(-y), 뒤 받침은 바깥(+y) -> 대차가 180° 돌면 셀 쪽 (cell_config.CART_SEAT_FACE)
    const box = el('g', { transform: `translate(${x},${y}) rotate(${GEO.cart_seat_face[s] < 0 ? 90 : -90})` }, cart);
    const { w } = drawCassette(box, 'in', 0);
    rect(-29.5, -70, 100, 140, 'clamp', box);                   // 클램프 윗판째로 실림
    R.seat[s] = { box, w, sr };
  }
  R.cartLbl = tx('대차', { x: -120, y: 130, class: 'stn-t', 'text-anchor': 'middle', 'dominant-baseline': 'central' }, cart);
  // 회전부(로봇)
  const rot = el('g', {}, g); R.rot = rot;
  circ(0, 0, 60, 'rob', rot);
  rect(-50, -50, 100, 100, 'rob2', rot, { 'fill-opacity': .35 });
  rect(-61.15, -21.15, 42.3, 42.3, 'rob2', rot);
  for (const [x, y] of [[31.82, -31.82], [-31.82, -31.82], [31.82, 31.82], [-31.82, 31.82]]) circ(x, y, 5, 'rob', rot);
  rect(-10, -17, 48, 34, 'rob2', rot);
  el('path', { d: 'M46.9 -2 L56 0 L46.9 2 Z', class: 'notch' }, rot);         // CL-02 노치(θ0 에서 포크 방향)
  R.rod = rect(38, -9, 58, 18, 'rob', rot);
  const blade = el('g', {}, rot); R.blade = blade;
  rect(96, -6, 11, 12, 'rob2', blade);
  rect(107, -20, 34, 40, 'rob2', blade);
  el('polygon', { points: '107,-30 131,-30 131,-36 149,-36 149,-34 203,-34 203,-20 149,-20 149,20 203,20 203,34 149,34 149,36 131,36 131,30 107,30', class: 'blade' }, blade);
  R.carry = circ(184.5, 0, 50, 'wafer carry', blade);
  R.carryCass = el('g', {}, blade);                        // 클램프 윗판째로 든 카세트(중심 = 포크 원점 +184.5)
  R.carryCassW = drawCassette(R.carryCass, 'in', 184.5).w;  // 입구 = 로봇 쪽(받침 · ① 과 같은 방향)
  rect(184.5 - 29.5, -70, 100, 140, 'clamp', R.carryCass);
  R.collide = circ(203, 0, 16, 'collide', blade);
  R.s4 = circ(GEO.s4_local[0], GEO.s4_local[1], 8, 'sen', blade);
  tx('S4', { x: GEO.s4_local[0], y: GEO.s4_local[1] + 3, class: 'sen-t', transform: '' }, blade);
  R.s4t = blade.lastChild;
  // θ 자리 센서 5 (고정, 허브 밑 링) - 로봇 위에 그려서 보이게
  R.ths = {};
  const tg = el('g', {}, g);
  circ(0, 0, 66, 'thring', tg);
  GEO.station_order.forEach((k, i) => { const [x, y] = pos(GEO.stations[k], 66); R.ths[GEO.th_sensors[i]] = circ(x, y, 6.5, 'ths', tg); });
  // 스테이션 이름(바깥쪽, 똑바로) - 회전하지 않는 겹에 화면 좌표로
  const lg = el('g', {}, svg);
  const lab = (k, r) => { const [x, y] = posV(GEO.stations[k], r); tx(ST_KR[k], { x, y, class: 'stn-t', 'text-anchor': 'middle', 'dominant-baseline': 'central' }, lg); };
  lab('uv', 364); lab('ng', 350); lab('inspect', 350); lab('input', 368);
  // 화면 좌표 = (-셀 y, -셀 x) (VIEW -90). 책상 글자는 화면 왼쪽 아래 안쪽 모서리
  tx('책상', { x: -dk.y1 + 10, y: -dk.x0 - 10, class: 'desk-t', 'text-anchor': 'start' }, lg);
}

/* ───────────────────────── 입면도 ───────────────────────── */
let PLATE_UNDER = 125;           // 카세트 바닥 -> 클램프 윗판 밑면(도크: 172 -> 297), init 에서 GEO 값으로
let SEAT_TOP = 163.5;            // 대차 받침 윗면 z, init 에서 GEO 값으로
function cassElev(parent, z0, x0) {
  // 카세트 단면(반경 215~300 + x0). z0 = 카세트 바닥(도크 172).
  const g = el('g', { transform: `translate(${x0 || 0},${-(z0 - 172)})` }, parent);
  rect(272, -290, 20, 118, 'post', g);
  rect(215, -190, 71, 10, 'part', g);
  rect(215, -180, 85, 8, 'part', g);
  for (let k = 2; k <= GEO.slot_count; k++) rect(215, -(GEO.slot_z0 + GEO.slot_pitch * (k - 1)), 71, 4, 'part', g);
  rect(215, -288, 77, 4, 'part', g);
  return g;
}
function slotWafers(parent, x0) {
  const out = [];
  for (let k = 1; k <= GEO.slot_count; k++) {
    const sk = GEO.slot_z0 + GEO.slot_pitch * (k - 1);
    out.push(rect(185.5 + x0, -(sk + GEO.wafer_t), 100, GEO.wafer_t, 'wafer', parent));
  }
  return out;
}
function buildElev() {
  const svg = $('elev');
  svg.innerHTML = '';
  const g = el('g', { id: 'elevScene' }, svg);
  rect(-95, -174, 460, 16, 'band det', g);                                   // 선회 대역(상세)
  rect(-60, -95, 120, 10, 'rob2', g);
  rect(-36.82, -372, 10, 285, 'rob2', g); rect(26.82, -372, 10, 285, 'rob2', g);
  rect(-44, -350, 8, 230, 'col', g);
  rect(-70, -374, 140, 10, 'rob2', g);
  rect(-61.15, -422, 42.3, 48, 'rob', g);
  R.st = {}; R.slotW = {};
  const dx = GEO.input_dock_shift;
  // ① 투입 카세트(+22) + 클램프 윗판 + TOF-IN
  R.st.input = el('g', {}, g);
  cassElev(R.st.input, 172, dx);
  R.slotW.input = slotWafers(R.st.input, dx);
  rect(205 + dx, -(172 + PLATE_UNDER + 6), 100, 6, 'clampE', R.st.input);
  rect(228.5 + dx, -146, 14, 6, 'tofs', R.st.input);
  // ③ 불량 카세트 + TOF-NG
  R.st.ng = el('g', {}, g);
  cassElev(R.st.ng, 172, 0);
  R.slotW.ng = slotWafers(R.st.ng, 0);
  rect(228.5, -146, 14, 6, 'tofs', R.st.ng);
  // ② 검사 크래들
  R.st.inspect = el('g', {}, g);
  rect(262, -250, 56, 6, 'hub', R.st.inspect); rect(220, -250, 45, 6, 'part', R.st.inspect);
  rect(218, -252, 8, 2, 'pad', R.st.inspect); rect(276.5, -252, 8, 2, 'pad', R.st.inspect);
  rect(276, -244, 20, 184, 'post', R.st.inspect);
  rect(238, -239, 30, 27, 'backlight', R.st.inspect);
  R.inspE = rect(185.5, -(GEO.cradle_seat_z + GEO.wafer_t), 100, GEO.wafer_t, 'wafer', R.st.inspect);
  // UV 탑 (받침 z183 · 위에 랜턴)
  R.st.uv = el('g', {}, g);
  const sz = GEO.uv_seat_z, ux = UR - 50;
  rect(ux - 6, -(sz + 1), 8, 4, 'pad', R.st.uv); rect(ux + 104, -(sz + 1), 8, 4, 'pad', R.st.uv);
  rect(ux + 108, -(sz + 110), 12, 110 + sz - 120, 'post', R.st.uv);
  R.uvBeam = rect(ux + 4, -(sz + 34), 92, 30, 'uvbeam', R.st.uv);
  rect(ux - 4, -(sz + 74), 108, 40, 'uvlamp', R.st.uv);
  R.uvE = rect(ux, -(sz + GEO.wafer_t), 100, GEO.wafer_t, 'wafer okw', R.st.uv);
  // UV 탑 밑 구간(RX > 2 에서 ZC 113.5 ~ 121.5 -> 포크 면 179 ~ 187) - 상세
  rect(ux - 10, -(GEO.uv_zc_in + GEO.blade_plane_ofs), 130, GEO.uv_zc_in - GEO.uv_zc_hand, 'uvzone det', R.st.uv);
  // 대차 받침(θ270, 카세트는 포크 R85 위치 = 반경 +34)
  R.st.cart = el('g', {}, g);
  rect(240, -SEAT_TOP, 100, 30, 'seat', R.st.cart);
  rect(228, -(SEAT_TOP - 30), 116, 60, 'cartbody', R.st.cart);
  R.st.cartCass = el('g', {}, R.st.cart);
  cassElev(R.st.cartCass, SEAT_TOP, 34);
  rect(205 + 34, -(SEAT_TOP + PLATE_UNDER + 6), 100, 6, 'clampE', R.st.cartCass);
  // 캐리지(Z=105 기준으로 그린 뒤 이동) + Sharp 로 잰 Z(상세, 점선)
  R.carMeas = rect(-50, -135, 100, 30, 'measbox', g);
  const car = el('g', {}, g); R.car = car;
  rect(-50, -135, 100, 30, 'rob', car);
  rect(-51, -140, 22, 5, 'rob2', car);
  rect(-10, -155, 48, 20, 'rob2', car);
  R.rodE = rect(38, -152, 58, 14, 'rob', car);
  const bl = el('g', {}, car); R.bladeE = bl;
  rect(96, -151, 11, 12, 'rob2', bl);
  rect(107, -165, 34, 14, 'rob2', bl);
  rect(107, -169, 96, 4, 'blade', bl);
  rect(149, -170.5, 8, 1.5, 'pad', bl); rect(193, -170.5, 8, 1.5, 'pad', bl);
  R.carryE = rect(134.5, -(170.5 + GEO.wafer_t), 100, GEO.wafer_t, 'wafer carry', bl);
  // 포크가 든 카세트: 윗판 밑면 = 포크 면(Z105 기준 z170.5), 반경은 RX 73 에서 집었으므로 -73
  R.carryCassE = el('g', {}, bl);
  const rxi = GEO.station_rx.input;
  cassElev(R.carryCassE, 170.5 - PLATE_UNDER, GEO.station_r.input - 235.5 - rxi);
  rect(205 + GEO.station_r.input - 235.5 - rxi, -176.5, 100, 6, 'clampE', R.carryCassE);
  R.s4E = circ(GEO.s4_local[0], -165, 3, 'sen', bl);
  R.collideE = circ(203, -167, 10, 'collide', bl);
}

/* ───────────────────────── 검사 카메라 개념도 ───────────────────────── */
function buildCam() {
  const svg = $('cam-sim');
  svg.innerHTML = '';
  const defs = el('defs', {}, svg);
  const lg = el('radialGradient', { id: 'bl', cx: '50%', cy: '50%', r: '65%' }, defs);
  el('stop', { offset: '0%', 'stop-color': '#fff8e6' }, lg); el('stop', { offset: '100%', 'stop-color': '#e9b35f' }, lg);
  rect(-120, -292, 240, 120, 'cambg', svg);
  R.camLight = rect(-58, -290, 116, 109, '', svg, { fill: 'url(#bl)' });
  // 백라이트 촬영은 빛 위의 실루엣만 보인다 -> 들어오는 웨이퍼/포크는 빛 칸 가장자리에서 나타나게
  const cp = el('clipPath', { id: 'camclip' }, defs);
  rect(-58, -290, 116, 109, '', cp);
  const inLight = el('g', { 'clip-path': 'url(#camclip)' }, svg);
  R.camTines = el('g', { opacity: .5 }, inLight);
  rect(-34, -210, 14, 54, '', R.camTines, { fill: '#2b3640' }); rect(20, -210, 14, 54, '', R.camTines, { fill: '#2b3640' });
  R.camW = el('circle', { cx: 0, cy: -235.5, r: 50, fill: '#101218', 'fill-opacity': .94, stroke: '#9c97d6', 'stroke-width': .8 }, inLight);
  el('path', { d: 'M-8 -235.5 H8 M0 -243.5 V-227.5', stroke: '#6fcf97', 'stroke-width': .6 }, svg);
  R.camFlash = rect(-120, -292, 240, 120, '', svg, { fill: '#fff', opacity: 0 });
}

/* ───────────────────────── 패널 골격 ───────────────────────── */
function buildStages() {
  const ol = $('stages'); ol.innerHTML = '';
  for (const [k, lbl] of META.stages) { const li = document.createElement('li'); li.dataset.stage = k; li.innerHTML = `<span class="t">${lbl}</span>`; ol.appendChild(li); }
}
function buildTimeline() {
  const tl = $('timeline'); tl.innerHTML = '';
  R.tc = {}; R.tg = {};
  const groups = [];
  for (const c of META.steps) {
    if (!groups.length || groups[groups.length - 1].g !== c.group) groups.push({ g: c.group, keys: [] });
    groups[groups.length - 1].keys.push(c.key);
  }
  for (const gr of groups) {
    const d = document.createElement('div'); d.className = 'tg';
    d.title = META.groups[gr.g] || gr.g; d.style.setProperty('--n', gr.keys.length);
    const cs = document.createElement('div'); cs.className = 'cs'; d.appendChild(cs);
    for (const k of gr.keys) {
      const st = CAT[k];
      const c = document.createElement('div'); c.className = 'tc'; c.title = `[${k}] ${st.title}\n${st.detail}`;
      c.textContent = k;
      const pg = document.createElement('span'); pg.className = 'pg'; pg.style.width = '0'; c.appendChild(pg);
      cs.appendChild(c); R.tc[k] = { c, pg, group: gr.g };
    }
    tl.appendChild(d); R.tg[gr.g] = d;
  }
}
function buildPanels() {
  const ax = $('axes'); ax.innerHTML = '';
  R.ax = {};
  const row = (k, name, lo, hi, unit, marks) => {
    const d = document.createElement('div'); d.className = 'ax';
    d.innerHTML = `<div class="an">${name}</div><div class="track"><span class="fill"></span><span class="ms" hidden></span><span class="tgt" hidden></span></div><div class="av">-</div>`;
    const tr = d.querySelector('.track');
    for (const m of marks) { const s = document.createElement('span'); s.className = 'mk'; s.style.left = `${(m - lo) / (hi - lo) * 100}%`; s.title = String(m); tr.appendChild(s); }
    ax.appendChild(d); R.ax[k] = { d, lo, hi, unit, fill: d.querySelector('.fill'), tgt: d.querySelector('.tgt'), ms: d.querySelector('.ms'), av: d.querySelector('.av') };
  };
  row('theta', 'θ', GEO.theta_frame[0], GEO.theta_frame[1], '°', Object.values(GEO.stations));
  row('zc', 'Z', GEO.zc_min, GEO.zc_soft_max, 'mm', [GEO.zc_swing, GEO.uv_zc_hand, GEO.uv_zc_in, GEO.cradle_zc_seat, GEO.cradle_zc_above, GEO.cass_z_seat, GEO.cass_z_under, GEO.cass_z_carry]);
  row('rx', 'R', -5, GEO.rx_max, 'mm', [GEO.rx_ret_ok, ...new Set(Object.values(GEO.station_rx))]);
  const chips = $('chips'); chips.innerHTML = '';
  R.chip = {};
  // θ 자리 센서 이름표는 설정(장소 각도)에서 만든다 - 장소를 옮겨도 따라감
  const STN_MARK = { ng: '③', inspect: '②', input: '①', uv: 'UV', cart: '대차' };
  const CH = [...GEO.station_order.map((k, i) => [GEO.th_sensors[i], `θ${GEO.stations[k]} ${STN_MARK[k] || k} 자리`]),
    ['S4', 'S4 포크 웨이퍼'], ['TOF_IN', 'TOF-IN ① 카세트'], ['TOF_NG', 'TOF-NG ③ 카세트'], ['UV', 'UV 램프 지령'], ['A_UV', 'A_UV LED 켜짐 확인'],
    ['D9', 'D9 E-stop 사슬 살아 있음'], ['ESTOP', '비상정지']];
  for (const [k, lbl] of CH) {
    const c = document.createElement('div'); c.className = 'chip' + (k === 'ESTOP' ? ' estop' : '');
    c.innerHTML = `<b>${k === 'ESTOP' ? 'E-STOP' : k.replace('_', '-')}</b>`; c.dataset.lbl = lbl; c.title = lbl;
    chips.appendChild(c); R.chip[k] = c;
  }
  // 하단: ① 투입(슬롯 5 가 위) · ③ 불량 · UV · 대차 받침 4
  const cb = $('cassettes'); cb.innerHTML = '';
  R.cbox = {};
  for (const k of ['input', 'ng']) {
    const d = document.createElement('div'); d.className = 'cas ' + k;
    d.innerHTML = `<div class="ch"><span class="nm">${k === 'input' ? '① 투입' : '③ 불량'}</span><b class="cnt">-</b></div><div class="cells"></div>`;
    const cells = d.querySelector('.cells'); const arr = [];
    for (let n = GEO.slot_count; n >= 1; n--) { const c = document.createElement('div'); c.className = 'cell'; c.textContent = n; cells.appendChild(c); arr[n] = c; }
    cb.appendChild(d); R.cbox[k] = { d, cnt: d.querySelector('.cnt'), cells: arr };
  }
  const ud = document.createElement('div'); ud.className = 'cas uvb';
  ud.innerHTML = `<div class="ch"><span class="nm">UV</span><b class="cnt">-</b></div><div class="uvcell"><span class="uvw"></span><b class="uvs"></b></div>`;
  cb.appendChild(ud); R.cbox.uv = { d: ud, cnt: ud.querySelector('.cnt'), cell: ud.querySelector('.uvcell'), s: ud.querySelector('.uvs') };
  const cd = document.createElement('div'); cd.className = 'cas cartb';
  cd.innerHTML = `<div class="ch"><span class="nm">대차</span><b class="cnt">-</b></div><div class="seats"></div>`;
  const seats = cd.querySelector('.seats'); R.cseat = {};
  for (const s of ['RL', 'RR', 'FL', 'FR']) { const c = document.createElement('div'); c.className = 'cseat'; c.textContent = GEO.cart_seat_kr[s]; seats.appendChild(c); R.cseat[s] = c; }
  cb.appendChild(cd); R.cbox.cart = { d: cd, cnt: cd.querySelector('.cnt') };
  const lgd = $('loadgrid'); lgd.innerHTML = '';
  for (let k = 1; k <= GEO.slot_count; k++) {
    const l = document.createElement('label'); l.innerHTML = `<input type="checkbox" data-slot="${k}" checked> 슬롯 ${k}`; lgd.appendChild(l);
  }
  const cg = $('cartgrid'); cg.innerHTML = '';
  for (const s of GEO.cart_seats) {
    const l = document.createElement('label'); l.innerHTML = `<input type="checkbox" data-seat="${s}" checked> ${GEO.cart_seat_kr[s]}`; cg.appendChild(l);
  }
  const fl = $('faults'); fl.innerHTML = '';
  R.fault = {};
  for (const f of META.faults) {
    const l = document.createElement('label'); l.innerHTML = `<input type="checkbox" data-fault="${f}"> ${FAULT_KR[f] || f}`;
    fl.appendChild(l); R.fault[f] = l;
    l.querySelector('input').addEventListener('change', (e) => cmd('fault', { name: f, on: e.target.checked }));
  }
  const asm = $('assumed'); asm.textContent = `가정 ${GEO.assumed.length}`; asm.title = '가정값 (실측 전): ' + GEO.assumed.join(', ');
}

/* ───────────────────────── 통신 ───────────────────────── */
let es = null, backoff = 500;
function connect() {
  es = new EventSource('/events');
  es.onmessage = (ev) => {
    let s;
    try { s = JSON.parse(ev.data); } catch (e) { return; }
    onSnap(s);
    backoff = 500;
  };
  es.onerror = () => {
    if (es.readyState === 2) { setTimeout(connect, backoff); backoff = Math.min(backoff * 2, 5000); }
  };
}
function onSnap(s) {
  const now = performance.now() / 1000;
  if (bootId && s.boot_id !== bootId) { buf.length = 0; offset = null; $('log').innerHTML = ''; }
  bootId = s.boot_id;
  lastMsgAt = now;
  const fresh = s.tick !== lastTick;
  if (fresh) { lastTick = s.tick; lastTickAt = now; }
  // 보간은 데이터 시각(t_src)으로: LIVE 는 ST(20 Hz)를 받은 시각이라 같은 ST 를 다시 보낸 스냅샷은 버퍼에 넣지 않는다
  const o = s.t_src - Date.now() / 1000;
  offset = offset === null ? o : Math.min(offset + 0.0005, o);
  if (fresh || !buf.length) { buf.push(s); while (buf.length > 8) buf.shift(); }
  if (s.new_log && s.new_log.length) addLog(s.new_log);
  snap = s;
  updatePanels(s);
}
async function cmd(c, args, opts) {
  if (BOOT.view_only || !BOOT.token) { toast('보기 전용 - 조작 불가'); return null; }
  const body = { cmd: c, args: args || {} };
  if (opts && opts.expect) body.expect_state = opts.expect;
  const ctl = new AbortController(); const tm = setTimeout(() => ctl.abort(), 4000);
  try {
    const r = await fetch('/api/cmd', { method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Wafer-Token': BOOT.token }, body: JSON.stringify(body), signal: ctl.signal });
    const j = await r.json();
    if (!j.ok) { handleCmdError(c, j, r.status); return j; }
    if (j.device) toast(`장비 수락: ${CMD_KR[c] || c}`);
    return j;
  } catch (e) { toast('명령 전송 실패 (서버 응답 없음)'); return null; } finally { clearTimeout(tm); }
}
function handleCmdError(c, j, status) {
  if (j.error === 'cell_not_clear' && isSim()) { confirmClear(); return; }
  if (status === 503) { toast(`장비 연결 안 됨 - ${CMD_KR[c] || c} 못 보냄`); return; }
  if (status === 504) { toast('장비 응답(ACK) 없음 - 상태를 확인하세요'); return; }
  toast(String(j.msg || j.error).replace(/\b([A-Z_]{4,})\b/g, (x) => STATE_KR[x] || x));
}

/* ───────────────────────── 프레임(보간) ───────────────────────── */
function interp() {
  if (!buf.length) return null;
  const lag = buf[buf.length - 1].source === 'SIM' ? 0.12 : 0.16;     // LIVE: ST 50 ms + SSE 50 ms 를 넘게
  const tNow = Date.now() / 1000 + (offset || 0) - lag;
  let a = buf[0], b = buf[buf.length - 1];
  if (tNow >= b.t_src) return b.pose;
  if (tNow <= a.t_src) return a.pose;
  for (let i = 0; i < buf.length - 1; i++) { if (buf[i].t_src <= tNow && tNow <= buf[i + 1].t_src) { a = buf[i]; b = buf[i + 1]; break; } }
  const f = (tNow - a.t_src) / Math.max(1e-6, b.t_src - a.t_src);
  return { theta: a.pose.theta + (b.pose.theta - a.pose.theta) * f, zc: a.pose.zc + (b.pose.zc - a.pose.zc) * f, rx: a.pose.rx + (b.pose.rx - a.pose.rx) * f };
}
function frame() {
  const p = interp();
  if (p && R.rot) drawPose(p);
  requestAnimationFrame(frame);
}
function nearStation(theta) {
  for (const [k, a] of Object.entries(GEO.stations)) { if (Math.abs(((theta - a + 540) % 360) - 180) <= 2) return k; }
  return null;
}
function drawPose(p) {
  R.rot.setAttribute('transform', `rotate(${-p.theta})`);
  R.blade.setAttribute('transform', `translate(${p.rx},0)`);
  R.rod.setAttribute('width', Math.max(4, 58 + p.rx));
  R.s4t.setAttribute('transform', `rotate(${p.theta - VIEW} ${GEO.s4_local[0]} ${GEO.s4_local[1]})`);
  R.car.setAttribute('transform', `translate(0,${-(p.zc - GEO.zc_swing)})`);
  R.bladeE.setAttribute('transform', `translate(${p.rx},0)`);
  R.rodE.setAttribute('width', Math.max(4, 58 + p.rx));
  let st = nearStation(p.theta);
  const ghost = !st;
  if (!st && snap && snap.target && snap.target.axis === 'theta') st = nearStation(snap.target.to);
  setElevStation(st, ghost);
  if (R.camTines) {
    const near = Math.abs(p.theta - GEO.stations.inspect) < 2;
    show(R.camTines, near);
    R.camTines.setAttribute('transform', `translate(0,${-p.rx})`);
    // 포크 위 웨이퍼는 빛 칸 안에 다 들어왔을 때만(R >= 약 47) 그린다 - 반쪽 원이 보이지 않게
    if (snap && snap.fork.wafer && near) { const cy = -(184.5 + p.rx); R.camW.setAttribute('cy', cy); show(R.camW, cy + 50 <= -181); }
  }
}
function setElevStation(st, ghost) {
  const on = { input: false, ng: false, inspect: false, uv: false, cart: false };
  if (st && snap) {
    if (st === 'input' || st === 'ng') {
      const d = snap.docks[st];
      on[st] = d.present;
      d.slots.forEach((x, i) => {
        const has = st === 'input' ? (x.code === 'U' || x.code === 'K') : x.code === 'N';
        show(R.slotW[st][i], has);
        R.slotW[st][i].setAttribute('class', 'wafer ' + (st === 'ng' ? 'ngw' : x.code === 'K' ? 'okw' : 'est'));
      });
    } else if (st === 'cart') {
      const a = snap.cart.aligned;
      on.cart = snap.cart.state !== 'away';
      show(R.st.cartCass, !!(a && snap.cart.seats[a] && snap.cart.seats[a] !== 'W'));
    } else on[st] = true;
  }
  for (const k in on) { show(R.st[k], on[k]); R.st[k].classList.toggle('ghoststn', ghost); }
}

/* ───────────────────────── 패널 갱신 ───────────────────────── */
function linkOk(s) { return s.source === 'SIM' || (s.link && s.link.status === 'OK'); }
function updatePanels(s) {
  const live = s.source !== 'SIM';
  document.body.classList.toggle('live', live);
  const sb = $('src-badge');
  sb.textContent = live ? '실장비' : '시뮬레이션';
  sb.className = 'srcbadge' + (live ? ' live' : '');
  sb.title = live ? '셀 Mega 데이터 (지령값 + 센서)' : '시뮬레이션 - 실제 장비 아님 · 사이클은 가정 속도 기준';
  const lb = $('link-badge'); const ls = (s.link || {}).status;
  lb.hidden = !live;
  const age = live && s.link.age_s !== null && s.link.age_s !== undefined ? Math.round(s.link.age_s * 1000) : null;
  lb.textContent = (LINK_KR[ls] || ls || '') + (ls === 'OK' && age !== null ? ` · ${age} ms` : '');
  lb.className = 'linkbadge ' + (ls === 'OK' ? 'ok' : ls === 'WAIT_HELLO' ? 'warn' : 'crit');
  if (live) { const L = s.link; lb.title = `${L.port || ''} · 마지막 장비 데이터(ST)를 받은 뒤 ${age === null ? '-' : age + ' ms'} (장비 -> PC 전송 지연은 잴 수 없음)${L.err ? ' · ' + L.err : ''}${L.fw ? ' · ' + L.fw : ''}`; }
  for (const x of document.querySelectorAll('.simonly')) x.hidden = live || BOOT.view_only;
  const stEl = $('st-state'); stEl.textContent = STATE_KR[s.state] || s.state; stEl.className = 's-' + s.state;
  for (const li of document.querySelectorAll('#stages li')) li.classList.toggle('on', li.dataset.stage === s.stage);
  for (const k in R.hi) R.hi[k].classList.toggle('on', s.step && nearStation(s.pose.theta) === k && RUNNING_STATES.includes(s.state));
  $('headline').textContent = s.headline || '';
  $('subline').textContent = s.subline || '';
  // ① · ③
  const c = s.counts;
  const di = s.docks.input, dn = s.docks.ng;
  show(R.cass.input.g, di.present);
  const inHas = di.slots.filter((x) => x.code === 'U' || x.code === 'K');
  show(R.cass.input.w, inHas.length > 0);
  R.cass.input.w.setAttribute('class', 'wafer ' + (inHas.length && inHas.every((x) => x.code === 'K') ? 'okw' : 'est'));
  show(R.cass.ng.g, dn.present); show(R.cass.ng.w, (dn.count || 0) > 0);
  const bi = R.cbox.input;
  bi.d.classList.toggle('absent', !di.present);
  bi.cnt.textContent = di.present ? `${di.slots.filter((x) => x.code === 'K').length} 원위치` : '없음';
  for (const x of di.slots) { const cell = bi.cells[x.n]; cell.className = 'cell ' + (CODE_CLS[x.code] || ''); cell.title = `슬롯 ${x.n}: ${CODE_KR[x.code] || x.code}`; }
  const bn = R.cbox.ng;
  bn.d.classList.toggle('absent', !dn.present);
  bn.cnt.textContent = dn.present ? `${dn.count}/${GEO.slot_count}` : '없음';
  for (const x of dn.slots) { const cell = bn.cells[x.n]; cell.className = 'cell ' + (x.code === 'N' ? 'ng' : ''); cell.title = x.code === 'N' ? `슬롯 ${x.n}: 불량` : `슬롯 ${x.n}: 비어 있음`; }
  // ② · UV
  show(R.inspW, s.inspect.wafer); show(R.inspE, s.inspect.wafer);
  R.inspW.setAttribute('class', 'wafer ' + (s.inspect.judgment === 'NG' ? 'ngw' : s.inspect.judgment === 'OK' ? 'okw' : ''));
  show(R.uvW, s.uvh.wafer); show(R.uvE, s.uvh.wafer);
  const lamp = !!s.uv.lamp;
  R.uvGlow.classList.toggle('on', lamp); R.uvBeam.classList.toggle('on', lamp);
  const bu = R.cbox.uv;
  bu.cell.className = 'uvcell' + (s.uvh.wafer ? ' w' : '') + (lamp ? ' on' : '');
  bu.s.textContent = lamp && s.uv.left_s !== null ? `${Math.ceil(s.uv.left_s)} s` : '';
  bu.cnt.textContent = lamp ? (s.uv.a_uv ? '조사 중' : 'A_UV ?') : (s.uvh.wafer ? '받침' : '-');
  bu.cell.title = `UV ${fmt(s.uv.total_s, 0)} s (가정) · A_UV ${s.uv.a_uv ? '켜짐' : '꺼짐'}`;
  // 포크
  show(R.carry, s.fork.wafer); show(R.carryE, s.fork.wafer);
  const jc = s.fork.judgment === 'NG' ? 'ngw' : s.fork.judgment === 'OK' ? 'okw' : '';
  R.carry.setAttribute('class', 'wafer carry ' + jc); R.carryE.setAttribute('class', 'wafer carry ' + jc);
  show(R.carryCass, s.fork.cassette); show(R.carryCassE, s.fork.cassette);
  // 대차
  const cart = s.cart;
  show(R.cart, cart.state !== 'away');
  show(R.cartGhost, cart.state === 'away');
  R.cart.setAttribute('transform', `translate(${cart.lat},${UR + cart.dist}) rotate(${cart.rot} 100 130)`);
  R.cart.classList.toggle('unknownpos', cart.pos_known === false);        // LIVE: 움직이는 동안 위치 데이터 없음
  R.cartLbl.setAttribute('transform', `rotate(${-cart.rot - VIEW} -120 130)`);
  let nF = 0, nD = 0;
  for (const [seat, code] of Object.entries(cart.seats)) {
    const on = code === 'F' || code === 'D';
    show(R.seat[seat].box, on);
    R.seat[seat].w.setAttribute('class', 'wafer ' + (code === 'D' ? 'okw' : ''));
    R.seat[seat].sr.setAttribute('class', 'seat' + (code === 'W' ? ' out' : ''));
    if (code === 'F') nF++; if (code === 'D') nD++;
    const cs = R.cseat[seat];
    cs.className = 'cseat' + (code ? ' ' + code : '') + (cart.aligned === seat && cart.state === 'docked' ? ' here' : '');
    cs.title = `${GEO.cart_seat_kr[seat]}: ${code ? SEAT_KR[code] : '빈 받침'}`;
  }
  R.cbox.cart.d.classList.toggle('absent', cart.state === 'away');
  R.cbox.cart.cnt.textContent = cart.state === 'away' ? '없음' : `끝 ${nD} · 남음 ${nF}`;
  $('st-done').textContent = c.inspected;
  $('st-okng').textContent = `OK ${c.ok} · NG ${c.ng}`;
  // 상세
  $('cnt-line').innerHTML = `원위치 <b>${c.ok}</b> · 불량 <b>${c.ng}</b> · 빈 슬롯 <b>${c.empty}</b> · ③ <b>${c.ng_dock === null ? '없음' : c.ng_dock + '/5'}</b> · 대차 출발 <b>${c.carts}</b>`;
  const j = s.judgment;
  $('judge-line').innerHTML = j ? `판정 웨이퍼 ${j.no} <b>${j.result}</b> · δr <b>${fmt(j.dr, 1)}</b> · δθ <b>${fmt(j.dth, 2)}</b>` : '판정 -';
  $('st-cycle').textContent = c.cycle_avg ? `${fmt(c.cycle_avg)} s` : '-';
  $('st-cycle').title = c.cycle_last ? `마지막 ${fmt(c.cycle_last)} s` : '';
  const m = s.meas;
  show(R.carMeas, m.zc_s !== null && m.zc_s !== undefined);
  if (m.zc_s !== null && m.zc_s !== undefined) R.carMeas.setAttribute('transform', `translate(0,${-(m.zc_s - GEO.zc_swing)})`);
  $('sharp-line').innerHTML = `Sharp Z <b>${fmt(m.zc_s)}</b> (${fmt(m.zd)} mm) · R <b>${fmt(m.rx_s)}</b> (${fmt(m.rd)} mm)`;
  $('uv-line').hidden = !(s.uv.missing && s.uv.missing.length);
  if (s.uv.missing && s.uv.missing.length) $('uv-line').textContent = 'UV 조건 미충족: ' + s.uv.missing.map((x) => META.uv_why[x] || x).join(', ');
  const L = s.link || {};
  $('link-line').hidden = !live;
  if (live) $('link-line').innerHTML = `연결 <b>${esc(L.port || '-')}</b> · ${LINK_KR[L.status] || L.status} · 부팅 ${esc(L.boot || '-')} · 깨진 줄 ${L.counters ? L.counters.bad_line + L.counters.bad_field : 0} · 빠진 ST ${L.counters ? L.counters.st_gap : 0} · 재시작 ${L.counters ? L.counters.reboot : 0}`;
  const col = s.alarm && s.alarm.code === 'COLLISION';
  R.collide.classList.toggle('on', !!col); R.collideE.classList.toggle('on', !!col);
  // 센서: θ 링 + S4 + 칩
  const almSen = s.alarm ? SENSOR_OF_ALARM[s.alarm.code] : null;
  for (const k of GEO.th_sensors) R.ths[k].setAttribute('class', 'ths' + (s.sensors[k] ? ' on' : '') + (s.th_low >= 2 || almSen === 'TH' ? ' alm' : '') + (s.expect && s.expect.sensor === k ? ' exp' : ''));
  R.s4.setAttribute('class', 'sen' + (s.sensors.S4 ? ' on' : '') + (s.expect && s.expect.sensor === 'S4' ? ' exp' : '') + (almSen === 'S4' ? ' alm' : ''));
  R.s4E.setAttribute('class', 'sen' + (s.sensors.S4 ? ' on' : ''));
  const sv = Object.assign({}, s.sensors, { UV: lamp });
  for (const k in R.chip) {
    const ch = R.chip[k]; ch.classList.toggle('on', !!sv[k]);
    const ex = s.expect && s.expect.sensor === k; ch.classList.toggle('expect', !!ex);
    ch.title = ch.dataset.lbl + (ex ? (s.expect.value ? ' · 기대 ON' : ' · 기대 OFF') : '');
  }
  const il = $('ilock');
  il.textContent = s.interlock.rotate_ok ? '선회 허용' : '선회 금지';
  il.className = 'pill' + (s.interlock.rotate_ok ? ' ok' : '');
  il.title = s.interlock.rotate_ok ? `선회 허용 (${s.interlock.mode})` : '선회 금지: ' + s.interlock.why.map((x) => META.rot_why[x] || x).join(', ');
  const homed = s.homed;
  $('homed-badge').hidden = homed;
  for (const k of ['theta', 'zc', 'rx']) {
    const a = R.ax[k]; const v = s.pose[k];
    const pct = (x) => `${Math.max(0, Math.min(100, (x - a.lo) / (a.hi - a.lo) * 100))}%`;
    a.fill.style.width = pct(v);
    const tg = s.target && s.target.axis === k ? s.target.to : null;
    a.tgt.hidden = tg === null; if (tg !== null) a.tgt.style.left = pct(tg);
    const mv = k === 'zc' ? m.zc_s : k === 'rx' ? m.rx_s : null;
    a.ms.hidden = mv === null || mv === undefined; if (!a.ms.hidden) a.ms.style.left = pct(mv);
    a.av.innerHTML = `${fmt(v, 1)}<small>${a.unit}</small>`;
    a.av.title = k === 'theta' ? '지령값 (θ 자리 센서로 확인)' : `지령값 · Sharp ${fmt(mv)} mm`;
    a.d.classList.toggle('unhomed', !homed);
  }
  updateCam(s);
  updateTimeline(s);
  renderAttention(s);
  updateControls(s);
  for (const b of document.querySelectorAll('#speed button')) b.classList.toggle('on', Number(b.dataset.speed) === s.speed);
  for (const f in R.fault) { const on = s.faults.includes(f); R.fault[f].classList.toggle('on', on); R.fault[f].querySelector('input').checked = on; }
  $('btn-ngnext').classList.toggle('on', !!s.ng_next);
  $('btn-auto').classList.toggle('on', !!s.operator.auto);
  const caps = $('caps'); const files = (s.captures || []).filter((x) => x.file);
  caps.hidden = !files.length;
  caps.innerHTML = files.slice().reverse().map((x) => `<span class="cap">웨이퍼 ${x.no} ${x.judgment || ''} <a href="/captures/${encodeURIComponent(x.file)}" target="_blank">jpg</a></span>`).join('');
}

// 알람 바 = 주의를 끄는 유일한 자리. 키가 바뀔 때만 다시 그린다.
function renderAttention(s) {
  let key = null, text = '', cls = 'alarmbar', title = '';
  const ls = (s.link || {}).status;
  if (s.source !== 'SIM' && ls !== 'OK') {
    text = `장비 ${LINK_KR[ls] || ls}` + (s.link.err ? ` (${s.link.err})` : '') + ' - 명령을 보낼 수 없음'; key = `L|${ls}|${s.link.err}`;
    if (ls === 'WAIT_HELLO') cls += ' wait';
  } else if (s.alarm) {
    const a = s.alarm; const msg = String(a.msg || a.code);
    text = msg + (a.step ? ` · 단계 ${a.step}` : '') + (a.fatal ? ' — 원점 복귀 필요' : '');
    title = a.code; key = `A|${a.code}|${a.t}|${text}`;
  } else if (s.state === 'PAUSED' && s.pause_reason === 'LINK') {
    text = s.headline; cls += ' wait'; key = 'P|LINK';
  } else if (s.state === 'WAIT_NG') {
    text = '③ 불량 카세트 교체 필요'; cls += ' wait'; key = 'W|NG';
  } else {
    const d = sharpGap(s);
    if (d) { text = d; cls += ' wait'; key = 'S|' + d; }
  }
  if (key === attnKey) return;
  attnKey = key;
  const ab = $('alarmbar');
  if (!key) { ab.hidden = true; return; }
  ab.hidden = false; ab.className = cls; ab.textContent = text; ab.title = title;
}
// Sharp 실측과 지령값 차이(축이 움직이는 중에는 Sharp 평균이 늦으므로 보지 않음)
function sharpGap(s) {
  if (!s.homed || s.state === 'HOMING') return null;
  const m = s.meas, p = s.pose, tol = GEO.sharp_tol, out = [];
  const moving = (ax) => s.target && s.target.axis === ax;
  if (m.zc_s !== null && m.zc_s !== undefined && !moving('zc') && Math.abs(m.zc_s - p.zc) > tol) out.push(`Z 실측 차 ${fmt(m.zc_s - p.zc)} mm (지령 ${fmt(p.zc)} · Sharp ${fmt(m.zc_s)})`);
  if (m.rx_s !== null && m.rx_s !== undefined && !moving('rx') && Math.abs(m.rx_s - p.rx) > tol) out.push(`R 실측 차 ${fmt(m.rx_s - p.rx)} mm (지령 ${fmt(p.rx)} · Sharp ${fmt(m.rx_s)})`);
  return out.length ? out.join(' · ') : null;
}
function updateTimeline(s) {
  const key = s.step ? s.step.key : null;
  const grp = s.step ? s.step.group : null;
  const keys = Object.keys(R.tc);
  const ci = keys.indexOf(key);
  for (const g in R.tg) R.tg[g].classList.toggle('cur', g === grp);
  keys.forEach((k, i) => {
    const t = R.tc[k];
    const isCur = k === key;
    t.c.classList.toggle('cur', isCur);
    t.c.classList.toggle('done', ci >= 0 && t.group === grp && i < ci);
    t.pg.style.width = isCur ? `${Math.round((s.step.action + 0.5) / s.step.actions * 100)}%` : '0';
  });
}
function updateCam(s) {
  const cm = s.cam || { mode: 'none' };
  const badge = $('cam-badge');
  const img = $('cam-img'), sim = $('cam-sim');
  if (cm.mode === 'live') {
    img.hidden = false; sim.style.display = 'none';
    badge.textContent = { OK: 'LIVE', STALE: '영상 정지', NO_SIGNAL: '신호 없음', FAIL: '카메라 실패' }[cm.status] || cm.status;
    badge.className = 'badge ' + (cm.status === 'OK' ? 'ok' : 'crit');
    if (cm.status === 'OK' && cm.seq !== camSeq && !camBusy) {
      camBusy = true; const nx = new Image();
      nx.onload = () => { img.src = nx.src; camSeq = cm.seq; camBusy = false; };
      nx.onerror = () => { camBusy = false; };
      nx.src = `/camera.jpg?seq=${cm.seq}`;
    }
  } else {
    img.hidden = true; sim.style.display = '';
    badge.textContent = cm.status === 'FAIL' ? '카메라 실패' : '개념도'; badge.className = 'badge' + (cm.status === 'FAIL' ? ' crit' : '');
    if (!s.fork.wafer) { R.camW.setAttribute('cy', -235.5); show(R.camW, s.inspect.wafer); }
    if (s.fork.wafer && Math.abs(s.pose.theta - GEO.stations.inspect) >= 2) show(R.camW, false);
    const capturing = s.step && s.step.key === 'I3' && s.state === 'RUNNING';
    R.camLight.setAttribute('opacity', capturing ? 1 : 0.35);
    R.camFlash.setAttribute('opacity', capturing ? 0.12 : 0);
  }
  // 판정 칩: 검사대 웨이퍼 또는 방금 판정해 포크에 든 웨이퍼
  const j = $('judge');
  const jr = s.inspect.judgment || (s.fork.wafer && s.fork.judgment) || null;
  j.hidden = !jr;
  if (jr) { j.textContent = jr; j.className = 'judge ' + (jr === 'NG' ? 'ng' : 'ok'); }
}
// 헤더 버튼: 지금 누를 수 있는 것만 고정 순서로, 다음 동작은 강조. 소프트 정지는 항상 맨 끝 같은 자리.
function updateControls(s) {
  const dis = stale || BOOT.view_only || !linkOk(s);
  const st = s.state;
  const sim = s.source === 'SIM';
  const pendNg = (s.operator.pending || []).some((p) => p.task === 'ng');
  const vis = {
    home: ALLOWED.home.includes(st), start: st === 'READY', resume: st === 'PAUSED',
    pause: ALLOWED.pause.includes(st), reset: ALLOWED.reset.includes(st),
    op_ng: sim && st === 'WAIT_NG' && pendNg && !s.operator.auto, stop: true,
  };
  let primary = null;
  if (st === 'IDLE') primary = 'home';
  else if (st === 'READY') primary = 'start';
  else if (st === 'PAUSED') primary = 'resume';
  else if (st === 'ALARM' || st === 'ESTOP') primary = 'reset';
  else if (st === 'WAIT_NG' && vis.op_ng) primary = 'op_ng';
  for (const b of document.querySelectorAll('#controls [data-cmd]')) {
    const k = b.dataset.cmd;
    b.hidden = !vis[k];
    b.disabled = dis || (k === 'stop' && !ALLOWED.stop.includes(st));
    b.classList.toggle('primary', k === primary);
  }
  $('btn-estop').disabled = dis || st === 'ESTOP';
  for (const id of ['btn-load', 'btn-cart', 'btn-fault', 'btn-ngnext', 'btn-auto']) $(id).disabled = dis;
  for (const b of document.querySelectorAll('#speed button')) b.disabled = dis;
}

/* ───────────────────────── 로그 ───────────────────────── */
function addLog(items) {
  const ol = $('log');
  for (const e of items) {
    const li = document.createElement('li'); li.className = e.level;
    li.innerHTML = `<time>${fmt(e.t, 1)}s</time><span>${esc(e.msg)}</span>`;
    ol.insertBefore(li, ol.firstChild);
  }
  while (ol.children.length > 300) ol.removeChild(ol.lastChild);
}

/* ───────────────────────── 모달/토스트 ───────────────────────── */
function modal(html, onBind) {
  $('modal-body').innerHTML = html; $('modal').hidden = false; modalOpen = true;
  const close = () => { $('modal').hidden = true; modalOpen = false; };
  for (const b of $('modal-body').querySelectorAll('[data-close]')) b.addEventListener('click', close);
  if (onBind) onBind(close);
  const go = $('modal-body').querySelector('.go'); if (go) go.focus();
}
const isSim = () => !snap || snap.source === 'SIM';
function confirmCmd(c) {
  const live = !isSim();
  const note = live ? `<p>실장비 · 지금 ${STATE_KR[snap.state] || snap.state}. 장비가 받으면 '장비 수락' 이 뜬다. 하드웨어 E-STOP 이 손 닿는 곳에 있는지 확인.</p>` : '';
  modal(`<h3>${CMD_KR[c] || c}</h3>${note}<div class="row"><button class="b" data-close>취소</button><button class="b go" id="m-ok">실행</button></div>`,
    (close) => { $('m-ok').addEventListener('click', () => { close(); cmd(c, {}, { expect: snap && snap.state }); }); });
}
function confirmClear() {
  const locs = [];
  if (snap.fork.wafer) locs.push(['blade', '포크 위 웨이퍼']);
  if (snap.inspect.wafer) locs.push(['inspect', '② 크래들 웨이퍼']);
  if (snap.uvh.wafer) locs.push(['uvh', 'UV 받침 웨이퍼']);
  if (snap.fork.cassette) locs.push(['cassette', '포크 위 카세트']);
  modal(`<h3>셀 안에 남은 것</h3><p>${locs.map((x) => x[1]).join(' · ') || '-'} · 손으로 치운 뒤 시작하세요.</p><div class="row"><button class="b" data-close>닫기</button>${locs.map((x) => `<button class="b go" data-rm="${x[0]}">(시뮬) ${x[1]} 치움</button>`).join('')}</div>`,
    (close) => { for (const b of $('modal-body').querySelectorAll('[data-rm]')) b.addEventListener('click', async () => { close(); await cmd('remove_wafer', { loc: b.dataset.rm }); }); });
}
let toastT = null;
function toast(m) { const t = $('toast'); t.textContent = m; t.hidden = false; clearTimeout(toastT); toastT = setTimeout(() => { t.hidden = true; }, 3200); }

/* ───────────────────────── 감시 ───────────────────────── */
function watchdog() {
  const now = performance.now() / 1000;
  let why = null;
  if (!lastMsgAt) why = '서버 연결 중…';
  else if (now - lastMsgAt > 1.5) why = '서버 연결 끊김';
  else if (now - lastTickAt > 1.5) why = isSim() ? '시뮬레이터 멈춤' : `장비 데이터 멈춤 (${LINK_KR[(snap.link || {}).status] || '-'})`;
  const was = stale; stale = !!why;
  $('stale').hidden = !stale; if (why) $('stale-why').textContent = why;
  if (was !== stale && snap) updateControls(snap);
}

/* ───────────────────────── 이벤트 연결 ───────────────────────── */
function closePops() { for (const x of document.querySelectorAll('.pop')) x.hidden = true; }
function bindUi() {
  for (const b of document.querySelectorAll('#controls [data-cmd], #pop-tools [data-cmd]')) {
    b.addEventListener('click', () => {
      const c = b.dataset.cmd;
      if (c === 'op_ng') return cmd('operator', { task: 'ng' });
      if (c === 'estop') closePops();
      if (!isSim() && ALLOWED[c]) return confirmCmd(c);                // 실장비: 6 개 명령 모두 확인 창
      if (SIM_CONFIRM.includes(c)) return confirmCmd(c);
      cmd(c, {}, { expect: snap && snap.state });
    });
  }
  for (const b of document.querySelectorAll('#speed button')) b.addEventListener('click', () => cmd('speed', { value: Number(b.dataset.speed) }));
  $('btn-ngnext').addEventListener('click', () => cmd('next_ng', { on: !(snap && snap.ng_next) }));
  $('btn-auto').addEventListener('click', () => cmd('auto_operator', { on: !(snap && snap.operator.auto) }));
  const pop = (id, btn) => $(btn).addEventListener('click', () => { const p = $(id); const open = p.hidden; closePops(); p.hidden = !open; });
  pop('pop-tools', 'btn-tools'); pop('pop-load', 'btn-load'); pop('pop-cart', 'btn-cart'); pop('pop-fault', 'btn-fault');
  for (const b of document.querySelectorAll('#pop-load [data-fill]')) b.addEventListener('click', () => {
    const f = b.dataset.fill; const boxes = document.querySelectorAll('#loadgrid input');
    boxes.forEach((x, i) => { x.checked = f === 'gap' ? [0, 2, 3].includes(i) : i < Number(f); });
  });
  $('load-apply').addEventListener('click', () => {
    const slots = [...document.querySelectorAll('#loadgrid input')].map((x) => x.checked);
    cmd('input_map', { slots }); $('pop-load').hidden = true; toast('다음 대차의 카세트부터 적용');
  });
  $('cart-apply').addEventListener('click', () => {
    const seats = [...document.querySelectorAll('#cartgrid input')].map((x) => x.checked);
    cmd('cart_map', { seats }); $('pop-cart').hidden = true; toast('다음 대차부터 적용');
  });
  $('btn-log').addEventListener('click', () => { closePops(); $('drawer').hidden = !$('drawer').hidden; });
  $('btn-log-close').addEventListener('click', () => { $('drawer').hidden = true; });
  $('btn-theme').addEventListener('click', () => {
    const t = document.body.dataset.theme === 'dark' ? 'light' : 'dark'; document.body.dataset.theme = t; store('wc-theme', t);
  });
  $('btn-detail').addEventListener('click', () => setDetail(!document.body.classList.contains('detail'), true));
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') { $('modal').hidden = true; modalOpen = false; closePops(); return; }
    if ((e.key === 'd' || e.key === 'D') && !e.ctrlKey && !e.altKey && !e.metaKey && !/^(INPUT|TEXTAREA|SELECT|BUTTON)$/.test(e.target.tagName) && !modalOpen) {
      setDetail(!document.body.classList.contains('detail'), true);
    }
  });
}
function setDetail(on, persist) {
  document.body.classList.toggle('detail', on); $('btn-detail').classList.toggle('on', on);
  if (persist) store('wc-detail', on ? '1' : '0');
}

async function init() {
  const th = store('wc-theme'); if (th) document.body.dataset.theme = th;
  const q = new URLSearchParams(location.search);
  if (q.get('theme')) document.body.dataset.theme = q.get('theme');
  if (BOOT.view_only) { $('viewonly').hidden = false; $('controls').hidden = true; for (const x of document.querySelectorAll('.simonly')) x.hidden = true; }
  try {
    META = await (await fetch('/api/meta')).json();
  } catch (e) { $('stale').hidden = false; $('stale-why').textContent = '서버에 연결할 수 없음'; return; }
  GEO = META.geometry;
  SR = GEO.station_r.inspect; IR = GEO.station_r.input; UR = GEO.station_r.uv;
  PLATE_UNDER = GEO.cass_plate_under; SEAT_TOP = GEO.cart_seat_top_z;
  for (const c of META.steps) CAT[c.key] = c;
  document.title = `${META.cell} 공정 모니터`;
  $('cellname').textContent = `웨이퍼 검사셀 ${META.cell}`;
  const qd = q.get('detail'), sd = store('wc-detail');
  setDetail(qd !== null ? qd === '1' : sd !== null ? sd === '1' : META.source !== 'SIM', false);
  buildPlan(); buildElev(); buildCam(); buildStages(); buildTimeline(); buildPanels(); bindUi();
  connect();
  setInterval(watchdog, 250);
  requestAnimationFrame(frame);
}
init();
