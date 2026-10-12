/* R9 웨이퍼 검사셀 공정 모니터 — 브라우저 쪽.
   서버(SSE 20Hz) 스냅샷을 받아 벽시계 기준으로 보간해 도면을 움직인다.
   평면도: 셀좌표(y 아래로 증가). θ 0°=+x(OK), 90°=위. 로봇 그룹 rotate(-θ). 스테이션 = ① 투입 180 · ② 검사 120 · ③ NG 60 · ④ OK 0,
   대차 받침 = θ -90(아래). 입면도 = 포크 방향 r-z 단면(포크는 항상 오른쪽), y = -z.
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

const STATE_KR = {
  IDLE: '원점 필요', HOMING: '원점 복귀 중', READY: '대기', RUNNING: '공정 진행',
  WAIT_INPUT: '투입 교체 대기', WAIT_OK: 'OK 카세트 대기', WAIT_NG: 'NG 카세트 대기', WAIT_CART: '대차 대기',
  WAIT_OPERATOR: '운전자 확인', PAUSED: '일시정지', ALARM: '알람', ESTOP: '비상정지',
};
const FAULT_KR = {
  s4_false_none: 'S4 오감지 (있는데 없음)', s4_false_present: 'S4 오감지 (놓은 뒤에도 있음)',
  s3_fail: 'S3 고장 (후진 미확인)', belt_skip: '벨트 이빨 튐 (S1 불일치)',
  camera_fail: '카메라 실패', sim_exception: '시뮬 내부 예외', cart_delay: '대차 호출 지연',
};
const RUNNING_STATES = ['HOMING', 'RUNNING', 'WAIT_INPUT', 'WAIT_OK', 'WAIT_NG', 'WAIT_CART'];
const ALLOWED = {
  home: ['IDLE', 'READY'], start: ['READY'], pause: RUNNING_STATES, resume: ['PAUSED'],
  stop: [...RUNNING_STATES, 'PAUSED', 'WAIT_OPERATOR'], reset: ['ALARM', 'ESTOP'],
  insp: ['IDLE', 'READY'], blade: ['IDLE', 'READY', 'ALARM'], prompt: ['WAIT_OPERATOR'], estop: null,
};
const CMD_KR = { home: '원점 복귀', start: '공정 시작', resume: '재개', reset: '알람 해제' };
const OP_OF_CMD = { op_input: 'input', op_ok: 'ok', op_ng: 'ng' };
const OP_LABEL = { input: '투입 카세트 교체', ok: '빈 OK 카세트 놓음', ng: '빈 NG 카세트 놓음' };
const SENSOR_OF_ALARM = { S3_NOT_CONFIRMED: 'S3', S4_REMAINS: 'S4', S4_MISSING: 'S4', S1_MISMATCH: 'S1', S2_NOT_FOUND: 'S2' };
const krState = (m) => String(m).replace(/\b(IDLE|HOMING|READY|RUNNING|WAIT_INPUT|WAIT_OK|WAIT_NG|WAIT_CART|WAIT_OPERATOR|PAUSED|ALARM|ESTOP)\b/g, (x) => STATE_KR[x]);
const ST_KR = { input: '투입', inspect: '검사', ng: 'NG', ok: 'OK', cart: '대차' };

let META = null, GEO = null;
const CAT = {};
let snap = null, bootId = null;
const buf = [];
let offset = null, lastMsgAt = 0, lastTick = -1, lastTickAt = 0, stale = false;
let logRing = [];
let camBusy = false, camSeq = 0;
let promptShown = null, modalOpen = false, attnKey = null, progSig = null;
const R = {};            // SVG 참조

/* ───────────────────────── 평면도 ───────────────────────── */
const SR = 235.5;                 // 스테이션 웨이퍼(카세트) 중심 반경
const SEAT_R = 269.5;             // 대차 받침 카세트 중심 반경(포크 R85)
const pos = (deg, r) => [r * Math.cos(deg * Math.PI / 180), -r * Math.sin(deg * Math.PI / 180)];
// 평면도 보기 회전: 대차(-90°, 셀 앞)를 화면 오른쪽에 둔다(넓은 패널에 대차 받침 4칸이 다 들어가게).
// 화면에서 OK 0° = 위, NG 60° = 왼쪽 위, 검사 120° = 왼쪽 아래, 투입 180° = 아래. 글자는 따로 똑바로 세운다.
const VIEW = -90;
const posV = (deg, r) => pos(deg - VIEW, r);

function drawCassette(parent, kind) {
  // 반경 방향 틀(입구가 로봇 쪽). CELL_V9 카세트 외형.
  const g = el('g', {}, parent);
  rect(250, -62, 50, 124, 'part2', g);
  rect(215, -62, 79, 5, 'part', g); rect(215, 57, 79, 5, 'part', g);
  rect(215, -57, 71, 10, 'part', g); rect(215, 47, 71, 10, 'part', g); rect(288, -57, 6, 114, 'part', g);
  rect(212, -65, 92, 130, 'cass-frame ' + kind, g);
  const w = circ(SR, 0, 50, 'wafer ' + (kind === 'ng' ? 'ngw' : kind === 'ok' ? 'okw' : ''), g);
  return { g, w };
}

function buildPlan() {
  const svg = $('plan');
  svg.innerHTML = '';
  const g = el('g', { transform: `rotate(${VIEW})` }, svg);
  el('path', { d: `M${-SR} 0 A${SR} ${SR} 0 0 1 ${SR} 0`, class: 'path' }, g);           // 웨이퍼 경로 180 -> 0
  el('path', { d: `M${SR} 0 A${SR} ${SR} 0 0 1 0 ${SEAT_R}`, class: 'path' }, g);        // OK 카세트 경로 0 -> -90
  R.hi = {};
  for (const [k, a] of Object.entries(GEO.stations)) { const [x, y] = pos(a, k === 'cart' ? SEAT_R : SR); R.hi[k] = circ(x, y, 66, 'stn-hi', g); }
  R.cass = {};
  for (const k of GEO.cassettes) {
    const sg = el('g', { transform: `rotate(${-GEO.stations[k]})` }, g);
    R.cass[k] = drawCassette(sg, k);
    if (k === 'ok') rect(206, -70, 100, 140, 'clamp', R.cass[k].g);
  }
  // 검사 크래들 120° (백라이트 C-03N 위 크래들 C-02N, 위에 StreamCam)
  const ig = el('g', { transform: `rotate(${-GEO.stations.inspect})` }, g);
  rect(262, -52, 56, 104, 'hub', ig); rect(220, -52, 45, 12, 'part', ig); rect(220, 40, 45, 12, 'part', ig);
  rect(238, -50, 30, 100, 'backlight', ig);
  for (const [x, y] of [[222, -43], [222, 43], [280.5, 0]]) circ(x, y, 4, 'pad', ig);
  R.inspW = circ(SR, 0, 50, 'wafer', ig);
  rect(SR - 14, -8.5, 28, 17, 'rob2', ig, { rx: 3 });
  // 고정부: 페데스탈, θ1 모터
  el('path', { d: 'M82.20,-21.65 A85,85 0 1,0 82.20,21.65 L33.5,21.65 L33.5,-21.65 Z', class: 'part2' }, g);
  rect(37.47, -21.15, 42.3, 42.3, 'part', g);
  // 대차(메카넘, 받침 2x2). 로컬: FL(0,0) FR(200,0) RL(200,260) RR(0,260), 중심(100,130), +y = 셀에서 멀어짐
  // 대차가 없을 때: 도킹 자리 윤곽(빈 오른쪽이 무엇을 위한 자리인지 보이게)
  R.cartGhost = el('g', { transform: `translate(0,${SEAT_R})` }, g);
  rect(-90, -80, 380, 420, 'seat ghostcart', R.cartGhost);
  const cart = el('g', {}, g); R.cart = cart;
  rect(-90, -80, 380, 420, 'cartbody', cart);
  for (const [x, y] of [[-90, -80], [250, -80], [-90, 300], [250, 300]]) rect(x, y, 40, 40, 'wheel', cart);
  R.seat = {};
  const SEAT_XY = { FL: [0, 0], FR: [200, 0], RL: [200, 260], RR: [0, 260] };
  for (const [s, [x, y]] of Object.entries(SEAT_XY)) {
    rect(x - 62, y - 62, 124, 124, 'seat', cart);
    const box = el('g', {}, cart);
    rect(x - 50, y - 50, 100, 100, 'cassbox', box);
    circ(x, y, 36, 'wafer okw', box);
    R.seat[s] = { box };
  }
  R.cartLbl = tx('대차', { x: -120, y: 130, class: 'stn-t', 'text-anchor': 'middle', 'dominant-baseline': 'central' }, cart);
  // 회전부(로봇)
  const rot = el('g', {}, g); R.rot = rot;
  circ(0, 0, 60, 'rob', rot);
  rect(-50, -50, 100, 100, 'rob2', rot, { 'fill-opacity': .35 });
  rect(-61.15, -21.15, 42.3, 42.3, 'rob2', rot);
  for (const [x, y] of [[31.82, -31.82], [-31.82, -31.82], [31.82, 31.82], [-31.82, 31.82]]) circ(x, y, 5, 'rob', rot);
  rect(-10, -17, 48, 34, 'rob2', rot);
  R.rod = rect(38, -9, 58, 18, 'rob', rot);
  const blade = el('g', {}, rot); R.blade = blade;
  rect(96, -6, 11, 12, 'rob2', blade);
  rect(107, -20, 34, 40, 'rob2', blade);
  el('polygon', { points: '107,-30 131,-30 131,-36 149,-36 149,-34 203,-34 203,-20 149,-20 149,20 203,20 203,34 149,34 149,36 131,36 131,30 107,30', class: 'blade' }, blade);
  R.carry = circ(184.5, 0, 50, 'wafer carry', blade);
  R.carryCass = el('g', {}, blade);                        // 클램프 윗판째로 든 OK 카세트(중심 = 포크 원점 +184.5)
  rect(184.5 - 50, -50, 100, 100, 'cassbox', R.carryCass);
  circ(184.5, 0, 36, 'wafer okw', R.carryCass);
  rect(184.5 - 55, -66, 110, 132, 'clamp', R.carryCass);
  R.collide = circ(203, 0, 16, 'collide', blade);
  // 센서
  R.sen = {};
  const sen = (id, x, y, parent) => { const gg = el('g', {}, parent); const c = circ(x, y, 9, 'sen', gg); const t = tx(id, { x, y: y + 3, class: 'sen-t' }, gg); R.sen[id] = { c, t, x, y }; };
  sen('S1', 72, 0, g); sen('S2', 0, 40, rot); sen('S3', 104, -30, rot); sen('S4', 140, 0, blade);
  R.sen.S1.t.setAttribute('transform', `rotate(${-VIEW} 72 0)`);
  // 스테이션 이름(바깥쪽, 똑바로) - 회전하지 않는 겹에 화면 좌표로
  const lg = el('g', {}, svg);
  const lab = (k, r) => { const [x, y] = posV(GEO.stations[k], r); tx(ST_KR[k], { x, y, class: 'stn-t', 'text-anchor': 'middle', 'dominant-baseline': 'central' }, lg); };
  lab('input', 336); lab('inspect', 348); lab('ng', 348); lab('ok', 336);
}

/* ───────────────────────── 입면도 ───────────────────────── */
const PLATE_UNDER = 125;         // 카세트 바닥 -> 클램프 윗판 밑면(도크: 172 -> 297)
const SEAT_TOP = 163.5;          // 대차 받침 윗면 z (Z223 에서 카세트가 받침에 닿는 높이로 잡음)
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
function buildElev() {
  const svg = $('elev');
  svg.innerHTML = '';
  const g = el('g', { id: 'elevScene' }, svg);
  rect(-95, -174, 460, 16, 'band det', g);
  rect(-60, -95, 120, 10, 'rob2', g);
  rect(-36.82, -372, 10, 285, 'rob2', g); rect(26.82, -372, 10, 285, 'rob2', g);
  rect(-44, -350, 8, 230, 'col', g);
  rect(-70, -374, 140, 10, 'rob2', g);
  rect(-61.15, -422, 42.3, 48, 'rob', g);
  R.st = {};
  // 카세트(투입/NG/OK 공용) + OK 클램프 윗판
  R.st.cass = el('g', {}, g);
  cassElev(R.st.cass, 172);
  R.slotW = [];
  for (let k = 1; k <= GEO.slot_count; k++) {
    const sk = GEO.slot_z0 + GEO.slot_pitch * (k - 1);
    R.slotW.push(rect(185.5, -(sk + GEO.wafer_t), 100, GEO.wafer_t, 'wafer', R.st.cass));
  }
  R.st.clamp = rect(205, -(172 + PLATE_UNDER + 6), 100, 6, 'clampE', R.st.cass);
  // 검사 크래들
  R.st.inspect = el('g', {}, g);
  rect(262, -250, 56, 6, 'hub', R.st.inspect); rect(220, -250, 45, 6, 'part', R.st.inspect);
  rect(218, -252, 8, 2, 'pad', R.st.inspect); rect(276.5, -252, 8, 2, 'pad', R.st.inspect);
  rect(276, -244, 20, 184, 'post', R.st.inspect);
  rect(238, -239, 30, 27, 'backlight', R.st.inspect);
  R.inspE = rect(185.5, -(252 + GEO.wafer_t), 100, GEO.wafer_t, 'wafer', R.st.inspect);
  // 대차 받침(θ -90, 카세트는 포크 R85 위치 = 반경 +34)
  R.st.cart = el('g', {}, g);
  rect(240, -SEAT_TOP, 100, 30, 'seat', R.st.cart);
  rect(228, -(SEAT_TOP - 30), 116, 60, 'cartbody', R.st.cart);
  R.st.cartCass = el('g', {}, R.st.cart);
  cassElev(R.st.cartCass, SEAT_TOP, 34);
  rect(205 + 34, -(SEAT_TOP + PLATE_UNDER + 6), 100, 6, 'clampE', R.st.cartCass);
  // 캐리지(Z=105 기준으로 그린 뒤 이동)
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
  // 포크가 든 OK 카세트: 윗판 밑면 = 포크 면(Z105 기준 z170.5), 반경은 R51 에서 집었으므로 -51
  R.carryCassE = el('g', {}, bl);
  cassElev(R.carryCassE, 170.5 - PLATE_UNDER, -51);
  rect(205 - 51, -176.5, 100, 6, 'clampE', R.carryCassE);
  R.s4E = circ(140, -165, 3, 'sen', bl);
  R.collideE = circ(203, -167, 10, 'collide', bl);
}

/* ───────────────────────── 검사 카메라 개념도 ───────────────────────── */
function buildCam() {
  const svg = $('cam-sim');
  svg.innerHTML = '';
  const defs = el('defs', {}, svg);
  const lg = el('radialGradient', { id: 'bl', cx: '50%', cy: '50%', r: '65%' }, defs);
  el('stop', { offset: '0%', 'stop-color': '#fff8e6' }, lg); el('stop', { offset: '100%', 'stop-color': '#e9b35f' }, lg);
  rect(-120, -292, 240, 120, '', svg, { fill: '#07090b' });
  R.camLight = rect(-58, -290, 116, 109, '', svg, { fill: 'url(#bl)' });
  // 백라이트 촬영은 빛 위의 실루엣만 보인다 -> 들어오는 웨이퍼/포크는 빛 칸 가장자리에서 나타나게(테두리에 잘려 보이지 않게)
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
function buildTimeline(keys) {
  const tl = $('timeline'); tl.innerHTML = '';
  R.tc = {};
  const groups = [];
  for (const k of keys) {
    const c = CAT[k] || { group: 'home' };
    if (!groups.length || groups[groups.length - 1].g !== c.group) groups.push({ g: c.group, keys: [] });
    groups[groups.length - 1].keys.push(k);
  }
  for (const gr of groups) {
    const d = document.createElement('div'); d.className = 'tg';
    d.title = META.groups[gr.g] || gr.g; d.style.setProperty('--n', gr.keys.length);
    const cs = document.createElement('div'); cs.className = 'cs'; d.appendChild(cs);
    for (const k of gr.keys) {
      const st = CAT[k] || { title: k, detail: '' };
      const c = document.createElement('div'); c.className = 'tc'; c.title = `[${k}] ${st.title}\n${st.detail}`;
      c.textContent = k;
      const pg = document.createElement('span'); pg.className = 'pg'; pg.style.width = '0'; c.appendChild(pg);
      cs.appendChild(c); R.tc[k] = { c, pg };
    }
    tl.appendChild(d);
  }
}
function buildPanels() {
  const ax = $('axes'); ax.innerHTML = '';
  R.ax = {};
  const row = (k, name, lo, hi, unit, marks) => {
    const d = document.createElement('div'); d.className = 'ax';
    d.innerHTML = `<div class="an">${name}</div><div class="track"><span class="fill"></span><span class="tgt" hidden></span></div><div class="av">-</div>`;
    const tr = d.querySelector('.track');
    for (const m of marks) { const s = document.createElement('span'); s.className = 'mk'; s.style.left = `${(m - lo) / (hi - lo) * 100}%`; s.title = String(m); tr.appendChild(s); }
    ax.appendChild(d); R.ax[k] = { d, lo, hi, unit, fill: d.querySelector('.fill'), tgt: d.querySelector('.tgt'), av: d.querySelector('.av') };
  };
  row('theta', 'θ1', GEO.theta_min - 5, GEO.theta_max + 5, '°', [-90, -40, 0, 60, 120, 180]);
  row('zc', 'Z', GEO.zc_min, GEO.zc_max, 'mm', [GEO.zc_swing, 183, 192, GEO.cart_z_under, GEO.cart_z_carry]);
  row('rx', 'R', 0, GEO.rx_max, 'mm', [GEO.rx_ext, GEO.rx_cart]);
  R.ax.rx.av.title = '지령값 · 전진 도달 센서 없음';
  const chips = $('chips'); chips.innerHTML = '';
  R.chip = {};
  for (const [k, lbl] of [['S1', 'θ1 원점'], ['S2', 'Z 원점'], ['S3', 'R 후진'], ['S4', '포크 웨이퍼'], ['ESTOP', '비상정지']]) {
    const c = document.createElement('div'); c.className = 'chip' + (k === 'ESTOP' ? ' estop' : '');
    c.innerHTML = `<b>${k === 'ESTOP' ? 'E-STOP' : k}</b>`; c.dataset.lbl = lbl; c.title = lbl;
    chips.appendChild(c); R.chip[k] = c;
  }
  // 하단: 카세트 3개(슬롯 5 가 위) + 대차 받침 4
  const cb = $('cassettes'); cb.innerHTML = '';
  R.cbox = {};
  for (const k of ['input', 'ng', 'ok']) {
    const d = document.createElement('div'); d.className = 'cas ' + k;
    d.innerHTML = `<div class="ch"><span class="nm">${ST_KR[k]}</span><b class="cnt">-</b></div><div class="cells"></div>`;
    const cells = d.querySelector('.cells'); const arr = [];
    for (let n = GEO.slot_count; n >= 1; n--) { const c = document.createElement('div'); c.className = 'cell'; c.textContent = n; cells.appendChild(c); arr[n] = c; }
    cb.appendChild(d); R.cbox[k] = { d, cnt: d.querySelector('.cnt'), cells: arr };
  }
  const cd = document.createElement('div'); cd.className = 'cas cartb';
  cd.innerHTML = `<div class="ch"><span class="nm">대차</span><b class="cnt">-</b></div><div class="seats"></div>`;
  const seats = cd.querySelector('.seats'); R.cseat = {};
  for (const s of ['RL', 'RR', 'FL', 'FR']) { const c = document.createElement('div'); c.className = 'cseat'; c.textContent = GEO.cart_seat_kr[s]; seats.appendChild(c); R.cseat[s] = c; }
  cb.appendChild(cd); R.cbox.cart = { d: cd, cnt: cd.querySelector('.cnt') };
  const lgd = $('loadgrid'); lgd.innerHTML = '';
  for (let k = 1; k <= GEO.slot_count; k++) {
    const l = document.createElement('label'); l.innerHTML = `<input type="checkbox" data-slot="${k}" checked> 슬롯 ${k}`; lgd.appendChild(l);
  }
  const fl = $('faults'); fl.innerHTML = '';
  R.fault = {};
  for (const f of META.faults) {
    const l = document.createElement('label'); l.innerHTML = `<input type="checkbox" data-fault="${f}"> ${FAULT_KR[f] || f}`;
    fl.appendChild(l); R.fault[f] = l;
    l.querySelector('input').addEventListener('change', (e) => cmd('fault', { name: f, on: e.target.checked }));
  }
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
  if (bootId && s.boot_id !== bootId) { buf.length = 0; logRing = []; offset = null; $('log').innerHTML = ''; }
  bootId = s.boot_id;
  lastMsgAt = now;
  if (s.tick !== lastTick) { lastTick = s.tick; lastTickAt = now; }
  const o = s.wall - Date.now() / 1000;
  offset = offset === null ? o : Math.min(offset + 0.0005, o);
  buf.push(s); while (buf.length > 6) buf.shift();
  if (s.new_log && s.new_log.length) addLog(s.new_log);
  snap = s;
  updatePanels(s);
}
async function cmd(c, args, opts) {
  if (BOOT.view_only || !BOOT.token) { toast('보기 전용 - 조작 불가'); return null; }
  const body = { cmd: c, args: args || {} };
  if (opts && opts.expect) body.expect_state = opts.expect;
  const ctl = new AbortController(); const tm = setTimeout(() => ctl.abort(), 3000);
  try {
    const r = await fetch('/api/cmd', { method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Wafer-Token': BOOT.token }, body: JSON.stringify(body), signal: ctl.signal });
    const j = await r.json();
    if (!j.ok) { handleCmdError(c, j); return j; }
    return j;
  } catch (e) { toast('명령 전송 실패 (서버 응답 없음)'); return null; } finally { clearTimeout(tm); }
}
function handleCmdError(c, j) {
  if (j.error === 'need_insp_confirm') { confirmInsp(); return; }
  if (j.error === 'blade_wafer') { confirmBlade(c === 'start'); return; }
  toast(krState(j.msg || j.error));
}

/* ───────────────────────── 프레임(보간) ───────────────────────── */
function interp() {
  if (!buf.length) return null;
  const tNow = Date.now() / 1000 + (offset || 0) - 0.12;
  let a = buf[0], b = buf[buf.length - 1];
  if (tNow >= b.wall) return b.pose;
  if (tNow <= a.wall) return a.pose;
  for (let i = 0; i < buf.length - 1; i++) { if (buf[i].wall <= tNow && tNow <= buf[i + 1].wall) { a = buf[i]; b = buf[i + 1]; break; } }
  const f = (tNow - a.wall) / Math.max(1e-6, b.wall - a.wall);
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
  R.rod.setAttribute('width', 58 + p.rx);
  for (const id of ['S2', 'S3', 'S4']) { const s = R.sen[id]; s.t.setAttribute('transform', `rotate(${p.theta - VIEW} ${s.x} ${s.y})`); }
  R.car.setAttribute('transform', `translate(0,${-(p.zc - GEO.zc_swing)})`);
  R.bladeE.setAttribute('transform', `translate(${p.rx},0)`);
  R.rodE.setAttribute('width', 58 + p.rx);
  let st = nearStation(p.theta);
  const ghost = !st;
  if (!st && snap && snap.target && snap.target.axis === 'theta') st = nearStation(snap.target.to);
  setElevStation(st, ghost);
  if (R.camTines) {
    const near = Math.abs(p.theta - GEO.stations.inspect) < 2;
    show(R.camTines, near);
    R.camTines.setAttribute('transform', `translate(0,${-p.rx})`);
    // 포크 위 웨이퍼는 빛 칸 안에 다 들어왔을 때만(R >= 약 47) 그린다 - 반쪽 원이 보이지 않게
    if (snap && snap.occ.blade && near) { const cy = -(184.5 + p.rx); R.camW.setAttribute('cy', cy); show(R.camW, cy + 50 <= -181); }
  }
}
function setElevStation(st, ghost) {
  const isCass = st === 'input' || st === 'ng' || st === 'ok';
  let cassOn = isCass, cartOn = st === 'cart';
  if (isCass && snap) {
    const d = snap.docks[st];
    cassOn = d.present;
    d.slots.forEach((x, i) => {
      show(R.slotW[i], x.status === 'wafer' || x.status === 'ok' || x.status === 'ng');
      R.slotW[i].setAttribute('class', 'wafer ' + (st === 'ng' ? 'ngw' : st === 'ok' ? 'okw' : ''));
    });
    show(R.st.clamp, st === 'ok');
  }
  if (cartOn && snap) {
    const a = snap.cart.aligned;
    cartOn = snap.cart.state !== 'away';
    show(R.st.cartCass, !!(a && snap.cart.seats[a] !== null));
  }
  show(R.st.cass, cassOn); show(R.st.inspect, st === 'inspect'); show(R.st.cart, cartOn);
  for (const k of ['cass', 'inspect', 'cart']) R.st[k].classList.toggle('ghoststn', ghost);
}

/* ───────────────────────── 패널 갱신 ───────────────────────── */
function updatePanels(s) {
  const live = s.source !== 'SIM';
  document.body.classList.toggle('live', live);
  const sb = $('src-badge');
  sb.textContent = live ? '실장비' : '시뮬레이션';
  sb.className = 'srcbadge' + (live ? ' live' : '');
  sb.title = live ? '실제 장비 데이터' : '시뮬레이션 - 실제 장비 아님 · 사이클은 가정 속도 기준';
  for (const x of document.querySelectorAll('.simonly')) x.hidden = live || BOOT.view_only;
  const stEl = $('st-state'); stEl.textContent = STATE_KR[s.state] || s.state; stEl.className = 's-' + s.state;
  // 흐름 띠 + 제목 한 줄
  for (const li of document.querySelectorAll('#stages li')) li.classList.toggle('on', li.dataset.stage === s.stage);
  for (const k in R.hi) R.hi[k].classList.toggle('on', k === s.stage);
  $('headline').textContent = s.headline || '';
  $('subline').textContent = s.subline || '';
  // 카세트 / 크래들 / 포크
  const c = s.counts;
  for (const k of ['input', 'ng', 'ok']) {
    const d = s.docks[k];
    show(R.cass[k].g, d.present);
    show(R.cass[k].w, d.count > 0);
    const bx = R.cbox[k];
    bx.d.classList.toggle('absent', !d.present);
    bx.cnt.textContent = d.present ? `${d.count}/${GEO.slot_count}` : '없음';
    for (const x of d.slots) {
      const cell = bx.cells[x.n];
      cell.className = 'cell ' + x.status;
      cell.title = x.no ? `웨이퍼 ${x.no}` : '';
    }
  }
  show(R.inspW, s.inspect.wafer);
  R.inspW.setAttribute('class', 'wafer ' + (s.inspect.judgment === 'NG' ? 'ngw' : s.inspect.judgment === 'OK' ? 'okw' : ''));
  show(R.inspE, s.inspect.wafer);
  show(R.carry, s.fork.wafer); show(R.carryE, s.fork.wafer);
  const jc = s.fork.judgment === 'NG' ? 'ngw' : s.fork.judgment === 'OK' ? 'okw' : '';
  R.carry.setAttribute('class', 'wafer carry ' + jc); R.carryE.setAttribute('class', 'wafer carry ' + jc);
  show(R.carryCass, s.fork.cassette !== null); show(R.carryCassE, s.fork.cassette !== null);
  // 대차
  const cart = s.cart;
  show(R.cart, cart.state !== 'away');
  show(R.cartGhost, cart.state === 'away');
  R.cart.setAttribute('transform', `translate(${cart.lat},${SEAT_R + cart.dist}) rotate(${cart.rot} 100 130)`);
  R.cartLbl.setAttribute('transform', `rotate(${-cart.rot - VIEW} -120 130)`);
  for (const [seat, n] of Object.entries(cart.seats)) {
    show(R.seat[seat].box, n !== null);
    R.cseat[seat].className = 'cseat' + (n !== null ? ' full' : '') + (cart.aligned === seat && cart.state === 'docked' ? ' here' : '');
  }
  R.cbox.cart.d.classList.toggle('absent', cart.state === 'away');
  R.cbox.cart.cnt.textContent = cart.state === 'away' ? '없음' : `${cart.loaded}/4`;
  $('st-done').textContent = c.inspected;
  $('st-done').title = `OK ${c.ok} · NG ${c.ng}`;
  // 상세
  $('cnt-input').textContent = `${c.input}/5`;
  $('cnt-ok').textContent = c.ok_dock === null ? '없음' : `${c.ok_dock}/5`;
  $('cnt-ng').textContent = c.ng_dock === null ? '없음' : `${c.ng_dock}/5`;
  $('cnt-cart').textContent = `${c.cart}/4`;
  $('cnt-okt').textContent = c.ok; $('cnt-ngt').textContent = c.ng; $('cnt-dep').textContent = cart.departed;
  $('st-cycle').textContent = c.cycle_avg ? `${fmt(c.cycle_avg)} s` : '-';
  $('st-cycle').title = c.cycle_last ? `마지막 ${fmt(c.cycle_last)} s` : '';
  $('cart-cam').textContent = cart.state === 'away' ? '-' : cart.camera;
  const col = s.alarm && s.alarm.code === 'COLLISION';
  R.collide.classList.toggle('on', !!col); R.collideE.classList.toggle('on', !!col);
  // 센서 LED + 칩
  const almSen = s.alarm ? SENSOR_OF_ALARM[s.alarm.code] : null;
  for (const k of ['S1', 'S2', 'S3', 'S4']) {
    const on = !!s.sensors[k];
    R.sen[k].c.setAttribute('class', 'sen' + (on ? ' on' : '') + (s.expect && s.expect.sensor === k ? ' exp' : '') + (almSen === k ? ' alm' : ''));
  }
  R.s4E.setAttribute('class', 'sen' + (s.sensors.S4 ? ' on' : ''));
  for (const k in R.chip) {
    const ch = R.chip[k]; ch.classList.toggle('on', !!s.sensors[k]);
    const ex = s.expect && s.expect.sensor === k; ch.classList.toggle('expect', !!ex);
    ch.title = ch.dataset.lbl + (ex ? (s.expect.value ? ' · 기대 ON' : ' · 기대 OFF') : '');
  }
  const il = $('ilock');
  il.textContent = s.interlock.rotate_ok ? '선회 허용' : '선회 금지';
  il.className = 'pill' + (s.interlock.rotate_ok ? ' ok' : '');
  il.title = s.interlock.reason;
  const homed = s.homed;
  $('homed-badge').hidden = homed;
  for (const k of ['theta', 'zc', 'rx']) {
    const a = R.ax[k]; const v = s.pose[k];
    a.fill.style.width = `${Math.max(0, Math.min(100, (v - a.lo) / (a.hi - a.lo) * 100))}%`;
    const tg = s.target && s.target.axis === k ? s.target.to : null;
    a.tgt.hidden = tg === null; if (tg !== null) a.tgt.style.left = `${(tg - a.lo) / (a.hi - a.lo) * 100}%`;
    a.av.innerHTML = `${fmt(v, 1)}<small>${a.unit}</small>`;
    a.av.classList.toggle('unver', homed && k === 'rx' && v > 0.5);
    a.d.classList.toggle('unhomed', !homed);
  }
  updateCam(s);
  updateTimeline(s);
  renderAttention(s);
  if (s.state === 'WAIT_OPERATOR' && s.prompt && !BOOT.view_only) {
    const key = `${s.prompt.kind}:${s.prompt.slot}:${s.log_seq}`;
    if (promptShown === null || !promptShown.startsWith(`${s.prompt.kind}:${s.prompt.slot}:`)) { promptShown = key; showPrompt(s.prompt); }
  } else if (s.state !== 'WAIT_OPERATOR') promptShown = null;
  updateControls(s);
  for (const b of document.querySelectorAll('#speed button')) b.classList.toggle('on', Number(b.dataset.speed) === s.speed);
  for (const f in R.fault) { const on = s.faults.includes(f); R.fault[f].classList.toggle('on', on); R.fault[f].querySelector('input').checked = on; }
  $('btn-ngnext').classList.toggle('on', !!s.ng_next);
  $('btn-auto').classList.toggle('on', !!s.operator.auto);
  const caps = $('caps'); const files = s.captures.filter((x) => x.file);
  caps.hidden = !files.length;
  caps.innerHTML = files.slice().reverse().map((x) => `<span class="cap">웨이퍼 ${x.no} ${x.judgment || ''} <a href="/captures/${encodeURIComponent(x.file)}" target="_blank">jpg</a></span>`).join('');
}
function esc(s) { return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])); }

// 알람 바 = 주의를 끄는 유일한 자리(알람 · 운전자 확인만). 키가 바뀔 때만 다시 그린다.
function renderAttention(s) {
  let key = null, text = '', cls = 'alarmbar', title = '';
  if (s.alarm) {
    const a = s.alarm; const msg = String(a.msg || a.code);
    text = msg + (a.step && !msg.includes('단계') ? ` · 단계 ${a.step}` : '') + (a.fatal ? ' — 원점 복귀 필요' : '');
    title = a.code; key = `A|${a.code}|${a.t}|${text}`;
  } else if (s.state === 'WAIT_OPERATOR' && s.prompt) {
    text = `투입 슬롯 ${s.prompt.slot} 확인 필요`; cls += ' wait'; key = `P|${s.prompt.slot}`;
  }
  if (key === attnKey) return;
  attnKey = key;
  const ab = $('alarmbar');
  if (!key) { ab.hidden = true; return; }
  ab.hidden = false; ab.className = cls; ab.textContent = text; ab.title = title;
}
function updateTimeline(s) {
  const keys = s.program || [];
  const sig = keys.join(',');
  if (sig !== progSig) { progSig = sig; buildTimeline(keys); }
  const key = s.step ? s.step.key : null;
  const ci = keys.indexOf(key);
  keys.forEach((k, i) => {
    const t = R.tc[k]; if (!t) return;
    const isCur = k === key;
    t.c.classList.toggle('cur', isCur);
    t.c.classList.toggle('done', ci >= 0 && i < ci);
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
    if (!s.occ.blade) { R.camW.setAttribute('cy', -235.5); show(R.camW, s.inspect.wafer); }
    if (s.occ.blade && Math.abs(s.pose.theta - GEO.stations.inspect) >= 2) show(R.camW, false);
    const capturing = s.step && s.step.key === '6' && s.state === 'RUNNING';
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
  const dis = stale || BOOT.view_only;
  const st = s.state;
  const pend = {}; for (const p of s.operator.pending) pend[p.task] = p;
  const vis = {
    home: ALLOWED.home.includes(st), start: st === 'READY', resume: st === 'PAUSED',
    pause: ALLOWED.pause.includes(st), prompt: st === 'WAIT_OPERATOR' && !!s.prompt,
    op_input: !!pend.input, op_ok: !!pend.ok, op_ng: !!pend.ng,
    reset: ALLOWED.reset.includes(st),
    insp: !s.insp_verified && ALLOWED.insp.includes(st),
    blade: (s.occ.blade || !!s.sensors.S4) && ALLOWED.blade.includes(st),
    stop: true,
  };
  let primary = null;
  if (st === 'IDLE') primary = 'home';
  else if (st === 'READY') primary = vis.blade ? 'blade' : vis.insp ? 'insp' : 'start';
  else if (st === 'PAUSED') primary = 'resume';
  else if (st === 'ALARM' || st === 'ESTOP') primary = 'reset';
  else if (st === 'WAIT_OPERATOR') primary = 'prompt';
  else if (st === 'WAIT_INPUT' && pend.input) primary = 'op_input';
  else if (st === 'WAIT_OK' && pend.ok) primary = 'op_ok';
  else if (st === 'WAIT_NG' && pend.ng) primary = 'op_ng';
  for (const b of document.querySelectorAll('#controls [data-cmd]')) {
    const k = b.dataset.cmd;
    b.hidden = !vis[k];
    b.disabled = dis || (k === 'stop' && !ALLOWED.stop.includes(st));
    b.classList.toggle('primary', k === primary);
    const task = OP_OF_CMD[k];
    if (task) {
      const p = pend[task];
      const label = p && p.due_in !== null ? `${OP_LABEL[task]} (자동 ${Math.ceil(p.due_in)}s)` : OP_LABEL[task];
      if (b.textContent !== label) b.textContent = label;
    }
  }
  $('btn-estop').disabled = dis || st === 'ESTOP';
  for (const id of ['btn-load', 'btn-fault', 'btn-ngnext', 'btn-auto']) $(id).disabled = dis;
  for (const b of document.querySelectorAll('#speed button')) b.disabled = dis;
}

/* ───────────────────────── 로그 ───────────────────────── */
function addLog(items) {
  const ol = $('log');
  for (const e of items) {
    logRing.push(e);
    const li = document.createElement('li'); li.className = e.level;
    li.innerHTML = `<time>${fmt(e.t, 1)}s</time><span>${esc(e.msg)}</span>`;
    ol.insertBefore(li, ol.firstChild);
  }
  while (logRing.length > 300) logRing.shift();
  while (ol.children.length > 300) ol.removeChild(ol.lastChild);
}

/* ───────────────────────── 모달/토스트 ───────────────────────── */
function modal(html, onBind) {
  $('modal-body').innerHTML = html; $('modal').hidden = false; modalOpen = true;
  const close = () => { $('modal').hidden = true; modalOpen = false; };
  for (const b of $('modal-body').querySelectorAll('[data-close]')) b.addEventListener('click', close);
  if (onBind) onBind(close);
}
const isSim = () => !snap || snap.source === 'SIM';
function liveNote() {
  return isSim() ? '' : `<p>실장비 · 현재 ${STATE_KR[snap.state] || snap.state}. 하드웨어 E-STOP 이 손 닿는 곳에 있는지 확인.</p>`;
}
function confirmCmd(c) {
  modal(`<h3>${CMD_KR[c] || c}</h3>${liveNote()}<div class="row"><button class="b" data-close>취소</button><button class="b go" id="m-ok">실행</button></div>`,
    (close) => { $('m-ok').addEventListener('click', () => { close(); cmd(c, {}, { expect: snap && snap.state }); }); });
}
function showPrompt(p) {
  modal(`<h3>투입 슬롯 ${p.slot}: 웨이퍼 감지 안 됨</h3><p>투입 카세트를 눈으로 확인하세요.</p><div class="row"><button class="b" id="m-retry">재시도 (웨이퍼 있음)</button><button class="b go" id="m-empty">빈 슬롯으로 처리</button></div>`,
    (close) => {
      $('m-retry').addEventListener('click', () => { close(); cmd('confirm', { choice: 'retry' }, { expect: 'WAIT_OPERATOR' }); });
      $('m-empty').addEventListener('click', () => { close(); cmd('confirm', { choice: 'empty' }, { expect: 'WAIT_OPERATOR' }); });
    });
}
function confirmInsp() {
  const sim = isSim();
  modal(`<h3>검사대 확인</h3><p>검사 크래들에 웨이퍼가 없는지 눈으로 확인하세요 (센서 없음).</p><div class="row"><button class="b" data-close>취소</button>${sim ? '<button class="b" id="m-rm">(시뮬) 웨이퍼 제거</button>' : ''}<button class="b go" id="m-clear">비어 있음 확인</button></div>`,
    (close) => {
      $('m-clear').addEventListener('click', () => { close(); cmd('confirm', { choice: 'insp_clear' }); });
      if (sim) $('m-rm').addEventListener('click', () => { cmd('remove_wafer', { loc: 'inspect' }); });
    });
}
function confirmBlade(thenStart) {
  const w = snap && snap.wafers.find((x) => x.loc === 'blade');
  const sim = isSim();
  if (!w) {
    modal(`<h3>S4 오감지</h3><p>포크에 웨이퍼가 없습니다.</p><div class="row"><button class="b" data-close>닫기</button>${sim ? '<button class="b go" id="m-f">오감지 끄기</button>' : ''}</div>`,
      (close) => { if (sim) $('m-f').addEventListener('click', () => { close(); cmd('fault', { name: 's4_false_present', on: false }); }); });
    return;
  }
  modal(`<h3>포크 위 웨이퍼${w.no ? ` (웨이퍼 ${w.no})` : ''}</h3><p>손으로 치운 뒤 누르세요.</p><div class="row"><button class="b" data-close>취소</button><button class="b go" id="m-rm">제거 완료${thenStart ? ' → 시작' : ''}</button></div>`,
    (close) => {
      $('m-rm').addEventListener('click', async () => {
        close();
        const r = await cmd('remove_wafer', { loc: 'blade' });
        if (r && r.ok && thenStart && snap && snap.state === 'READY') cmd('start', {}, { expect: snap.state });
      });
    });
}
let toastT = null;
function toast(m) { const t = $('toast'); t.textContent = m; t.hidden = false; clearTimeout(toastT); toastT = setTimeout(() => { t.hidden = true; }, 3200); }

/* ───────────────────────── 감시 ───────────────────────── */
function watchdog() {
  const now = performance.now() / 1000;
  let why = null;
  if (!lastMsgAt) why = '서버 연결 중…';
  else if (now - lastMsgAt > 1.5) why = '서버 연결 끊김';
  else if (now - lastTickAt > 1.5) why = isSim() ? '시뮬레이터 멈춤' : '장비 데이터 멈춤';
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
      if (c === 'insp') return confirmInsp();
      if (c === 'blade') return confirmBlade(false);
      if (c === 'prompt') return snap && snap.prompt && showPrompt(snap.prompt);
      if (OP_OF_CMD[c]) return cmd('operator', { task: OP_OF_CMD[c] });
      if (c === 'estop') closePops();
      if (b.dataset.confirm) return confirmCmd(c);
      cmd(c, {}, { expect: snap && snap.state });
    });
  }
  for (const b of document.querySelectorAll('#speed button')) b.addEventListener('click', () => cmd('speed', { value: Number(b.dataset.speed) }));
  $('btn-ngnext').addEventListener('click', () => cmd('next_ng', { on: !(snap && snap.ng_next) }));
  $('btn-auto').addEventListener('click', () => cmd('auto_operator', { on: !(snap && snap.operator.auto) }));
  const pop = (id, btn) => $(btn).addEventListener('click', () => { const p = $(id); const open = p.hidden; closePops(); p.hidden = !open; });
  pop('pop-tools', 'btn-tools'); pop('pop-load', 'btn-load'); pop('pop-fault', 'btn-fault');
  for (const b of document.querySelectorAll('#pop-load [data-fill]')) b.addEventListener('click', () => {
    const f = b.dataset.fill; const boxes = document.querySelectorAll('#loadgrid input');
    boxes.forEach((x, i) => { x.checked = f === 'gap' ? [0, 2, 3].includes(i) : i < Number(f); });
  });
  $('load-apply').addEventListener('click', () => {
    const slots = [...document.querySelectorAll('#loadgrid input')].map((x) => x.checked);
    cmd('load_input', { slots }); $('pop-load').hidden = true;
  });
  $('btn-log').addEventListener('click', () => { closePops(); $('drawer').hidden = !$('drawer').hidden; });
  $('btn-log-close').addEventListener('click', () => { $('drawer').hidden = true; });
  $('btn-theme').addEventListener('click', () => {
    const t = document.body.dataset.theme === 'dark' ? 'light' : 'dark'; document.body.dataset.theme = t; store('wc-theme', t);
  });
  $('btn-detail').addEventListener('click', () => setDetail(!document.body.classList.contains('detail'), true));
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') { $('modal').hidden = true; modalOpen = false; closePops(); return; }
    if ((e.key === 'd' || e.key === 'D') && !e.ctrlKey && !e.altKey && !e.metaKey && !/^(INPUT|TEXTAREA|SELECT)$/.test(e.target.tagName)) {
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
  for (const c of META.steps) CAT[c.key] = c;
  CAT.W = { group: 'home', title: '대기로 복귀', detail: 'Z 105 · R 0' };
  const qd = q.get('detail'), sd = store('wc-detail');
  setDetail(qd !== null ? qd === '1' : sd !== null ? sd === '1' : META.source !== 'SIM', false);
  buildPlan(); buildElev(); buildCam(); buildPanels(); bindUi();
  connect();
  setInterval(watchdog, 250);
  requestAnimationFrame(frame);
}
init();
