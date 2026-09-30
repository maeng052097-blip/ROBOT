/* CELL_V9 공정 모니터 — 브라우저 쪽.
   서버(SSE 20Hz) 스냅샷을 받아 벽시계 기준으로 보간해 도면을 움직인다.
   도면 좌표 = CELL_V9 문서 SVG 좌표 그대로(y 가 아래로 증가). 셀 +y(90°) 는 화면 위쪽 -> 로봇 그룹 rotate(-θ).
   입면도 = 블레이드 방향 r-z 단면(블레이드는 항상 오른쪽), y = -z.
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

const STATE_KR = {
  IDLE: '원점 필요', HOMING: '원점 복귀 중', READY: '준비 완료', RUNNING: '공정 진행',
  WAIT_S5: '인계 대기', WAIT_OPERATOR: '운전자 확인', PAUSED: '일시정지', ALARM: '알람',
  ESTOP: '비상정지', CASSETTE_DONE: '카세트 완료',
};
const SLOT_KR = { unknown: '?', est_present: '추정', present: '확인', moving: '이송', inspect: '검사',
  output: '반출', done: '완료', empty: '빈칸', above_top: '-', removed: '제거' };
const FAULT_KR = {
  s4_false_none: 'S4 오감지 (있는데 없음)', s4_false_present: 'S4 오감지 (안착 후에도 있음)',
  s3_fail: 'S3 고장 (후진 미확인)', belt_skip: '벨트 이빨 튐 (S1 불일치)',
  next_robot_delay: '다음 이송로봇 지연', camera_fail: '카메라 실패',
  sim_exception: '시뮬 내부 예외', tof_ledge_in_beam: 'TOF 빔이 선반에 걸림',
};
const ALLOWED = {
  home: ['IDLE', 'READY', 'CASSETTE_DONE'], cassette_ready: ['IDLE', 'READY', 'CASSETTE_DONE'],
  start: ['READY', 'CASSETTE_DONE'], pause: ['HOMING', 'RUNNING', 'WAIT_S5'], resume: ['PAUSED'],
  stop: ['HOMING', 'RUNNING', 'WAIT_S5', 'PAUSED', 'WAIT_OPERATOR'], reset: ['ALARM', 'ESTOP'],
  c90: ['IDLE', 'READY', 'CASSETTE_DONE'], blade: ['IDLE', 'READY', 'CASSETTE_DONE', 'ALARM'], prompt: ['WAIT_OPERATOR'],
  estop: null,
};
// 알람이 지목하는 센서 -> 평면도 LED 에 빨간 링 (코드명 = sim.py ALARM_TEXT)
const SENSOR_OF_ALARM = { S3_NOT_CONFIRMED: 'S3', S4_REMAINS: 'S4', S4_MISSING: 'S4', S1_MISMATCH: 'S1', S2_NOT_FOUND: 'S2' };
const krState = (m) => String(m).replace(/\b(IDLE|HOMING|READY|RUNNING|WAIT_S5|WAIT_OPERATOR|PAUSED|ALARM|ESTOP|CASSETTE_DONE)\b/g, (x) => STATE_KR[x]);
const CMD_KR = { home: '원점 복귀', start: '공정 시작', resume: '재개', reset: '알람 해제' };

let META = null, GEO = null;
let snap = null, bootId = null;
const buf = [];
let offset = null, lastMsgAt = 0, lastTick = -1, lastTickAt = 0, stale = false;
let logRing = [];
let camBusy = false, camSeq = 0;
let promptShown = null, modalOpen = false;
let s5Late = false, attnKey = null;   // 인계 지연(S5_WAIT 경고 뒤에만 알람 바), 알람 바 재그리기 키
const R = {};            // SVG 참조

/* ───────────────────────── 평면도 ───────────────────────── */
function buildPlan() {
  const svg = $('plan');
  svg.innerHTML = '';
  const g = el('g', {}, svg);
  rect(-345, -63.03, 690, 20, 'rail', g); rect(-345, 43.03, 690, 20, 'rail', g);
  el('path', { d: 'M235.5 0 A235.5 235.5 0 0 0 -235.5 0', class: 'path' }, g);   // 웨이퍼 이동 경로(0->90->180)
  for (const [x, y] of [[272, -50], [272, 30], [272, -85], [272, 65], [-40, -296], [20, -296], [-50, -330], [30, -330], [-296, -40], [-296, 20]]) rect(x, y, 20, 20, 'post', g);
  // 스테이션 강조 링
  R.hi = {
    cassette: circ(235.5, 0, 60, 'stn-hi', g), inspect: circ(0, -235.5, 60, 'stn-hi', g), output: circ(-235.5, 0, 60, 'stn-hi', g),
  };
  // 카세트 0°
  rect(250, -62, 50, 124, 'part2', g); rect(250, -88, 42, 26, 'part2', g); rect(250, 62, 42, 26, 'part2', g);
  rect(215, -62, 79, 5, 'part', g); rect(215, 57, 79, 5, 'part', g);
  rect(215, -57, 71, 10, 'part', g); rect(215, 47, 71, 10, 'part', g); rect(288, -57, 6, 114, 'part', g);
  R.casW = circ(235.5, 0, 50, 'wafer', g);
  // 검사 90°
  rect(-52, -318, 104, 56, 'hub', g); rect(-52, -265, 12, 45, 'part', g); rect(40, -265, 12, 45, 'part', g);
  rect(-50, -268, 100, 30, 'backlight', g);
  R.c90W = circ(0, -235.5, 50, 'wafer', g);
  for (const [x, y] of [[-43, -222], [43, -222], [0, -280.5]]) circ(x, y, 4, 'pad', g);
  rect(-20, -330, 40, 90, 'dash', g);
  // 반출 180°
  rect(-318, -52, 56, 104, 'hub', g); rect(-265, -52, 45, 12, 'part', g); rect(-265, 40, 45, 12, 'part', g);
  R.c180W = circ(-235.5, 0, 50, 'wafer', g);
  for (const [x, y] of [[-222, -43], [-222, 43], [-280.5, 0]]) circ(x, y, 4, 'pad', g);
  // 고정부: 페데스탈, θ1 모터, 20T 풀리
  el('path', { d: 'M82.20,-21.65 A85,85 0 1,0 82.20,21.65 L33.5,21.65 L33.5,-21.65 Z', class: 'part2' }, g);
  rect(37.47, -21.15, 42.3, 42.3, 'part', g);
  circ(58.62, 0, 6.37, 'dash', g);
  // 회전부
  const rot = el('g', {}, g); R.rot = rot;
  circ(0, 0, 60, 'rob', rot);
  rect(-50, -50, 100, 100, 'rob2', rot, { 'fill-opacity': .35 });
  rect(-61.15, -21.15, 42.3, 42.3, 'rob2', rot);
  for (const [x, y] of [[31.82, -31.82], [-31.82, -31.82], [31.82, 31.82], [-31.82, 31.82]]) circ(x, y, 5, 'rob', rot);
  circ(-40, 0, 4, 'rob', rot);
  rect(-10, -17, 48, 34, 'rob2', rot);
  R.rod = rect(38, -9, 58, 18, 'rob', rot);
  const blade = el('g', {}, rot); R.blade = blade;
  rect(96, -6, 11, 12, 'rob2', blade);
  rect(107, -20, 34, 40, 'rob2', blade);
  el('polygon', { points: '107,-30 131,-30 131,-36 149,-36 149,-34 203,-34 203,-20 149,-20 149,20 203,20 203,34 149,34 149,36 131,36 131,30 107,30', class: 'blade' }, blade);
  R.carry = circ(184.5, 0, 50, 'wafer carry', blade);
  R.collide = circ(203, 0, 16, 'collide', blade);
  // 센서
  R.sen = {};
  const sen = (id, x, y, parent) => { const gg = el('g', {}, parent); const c = circ(x, y, 9, 'sen', gg); const t = tx(id, { x, y: y + 3, class: 'sen-t' }, gg); R.sen[id] = { c, t, x, y }; };
  sen('S1', -72, 0, g); sen('S5', -265, 0, g); sen('S2', 0, 40, rot); sen('S3', 104, -30, rot); sen('S4', 140, 0, blade);
  rect(-14, -259, 28, 17, 'rob2', g, { rx: 3 });   // 검사 카메라
  // 스테이션 이름만(설명은 발표자가 한다)
  const lab = (x, y, t) => tx(t, { x, y, class: 'stn-t', 'text-anchor': 'middle' }, g);
  lab(262, -100, '카세트');
  lab(120, -300, '검사');
  lab(-262, -72, '반출');
}

/* ───────────────────────── 입면도 ───────────────────────── */
function buildElev() {
  const svg = $('elev');
  svg.innerHTML = '';
  const g = el('g', { id: 'elevScene' }, svg);
  // 선회 대역(상세에서만)
  rect(-95, -174, 430, 16, 'band det', g);
  // 고정 기둥(회전부 포함, 단면에선 정지로 보임)
  rect(-60, -95, 120, 10, 'rob2', g);
  rect(-36.82, -372, 10, 285, 'rob2', g); rect(26.82, -372, 10, 285, 'rob2', g);
  rect(-44, -350, 8, 230, 'col', g);
  rect(-70, -374, 140, 10, 'rob2', g);
  rect(-61.15, -422, 42.3, 48, 'rob', g);
  // 스테이션 단면(오른쪽)
  R.st = {};
  R.st.cassette = el('g', {}, g);
  const cg = R.st.cassette;
  rect(272, -302, 20, 242, 'post', cg);
  rect(215, -310, 79, 130, 'dash', cg);
  rect(250, -180, 50, 8, 'part', cg);
  rect(215, -190, 71, 10, 'part', cg);
  for (let k = 2; k <= GEO.slot_count; k++) { const sk = GEO.slot_z0 + GEO.slot_pitch * (k - 1); rect(215, -sk, 71, 4, 'part', cg); }
  rect(215, -310, 71, 4, 'part', cg);
  R.slotW = [];
  for (let k = 1; k <= GEO.slot_count; k++) {
    const sk = GEO.slot_z0 + GEO.slot_pitch * (k - 1);
    R.slotW.push(rect(185.5, -(sk + GEO.wafer_t), 100, GEO.wafer_t, 'wafer', cg));
  }
  const cradle = (parent, withS5) => {
    rect(262, -250, 56, 6, 'hub', parent); rect(220, -250, 45, 6, 'part', parent);
    rect(218, -252, 8, 2, 'pad', parent); rect(276.5, -252, 8, 2, 'pad', parent);
    rect(276, -244, 20, 184, 'post', parent);
    const w = rect(185.5, -(252 + GEO.wafer_t), 100, GEO.wafer_t, 'wafer', parent);
    if (withS5) return [w, circ(265, -238, 6, 'sen', parent)];
    return [w, null];
  };
  R.st.inspect = el('g', {}, g);
  [R.c90Ew] = cradle(R.st.inspect, false);
  rect(238, -239, 30, 27, 'backlight', R.st.inspect);
  R.st.output = el('g', {}, g);
  [R.c180Ew, R.s5E] = cradle(R.st.output, true);
  // 캐리지(ZC=105 기준으로 그린 뒤 이동)
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
  R.s4E = circ(140, -165, 3, 'sen', bl);
  R.collideE = circ(203, -167, 10, 'collide', bl);
}

/* ───────────────────────── 검사 카메라 개념도 ───────────────────────── */
function buildCam() {
  const svg = $('cam-sim');
  svg.setAttribute('viewBox', '-120 -292 240 120');
  svg.innerHTML = '';
  const defs = el('defs', {}, svg);
  const lg = el('radialGradient', { id: 'bl', cx: '50%', cy: '50%', r: '65%' }, defs);
  el('stop', { offset: '0%', 'stop-color': '#fff8e6' }, lg); el('stop', { offset: '100%', 'stop-color': '#e9b35f' }, lg);
  rect(-120, -292, 240, 120, '', svg, { fill: '#07090b' });
  rect(-52, -292, 104, 30, '', svg, { fill: '#1b2229' });
  rect(-52, -265, 12, 45, '', svg, { fill: '#161c22' }); rect(40, -265, 12, 45, '', svg, { fill: '#161c22' });
  R.camLight = rect(-44, -262, 88, 18, '', svg, { fill: 'url(#bl)' });
  R.camGlow = rect(-50, -268, 100, 30, '', svg, { fill: 'none', stroke: '#e9b35f', 'stroke-opacity': .35 });
  R.camTines = el('g', { opacity: .5 }, svg);
  rect(-34, -210, 14, 54, '', R.camTines, { fill: '#2b3640' }); rect(20, -210, 14, 54, '', R.camTines, { fill: '#2b3640' });
  R.camW = el('circle', { cx: 0, cy: -235.5, r: 50, fill: '#101218', 'fill-opacity': .94, stroke: '#9c97d6', 'stroke-width': .8 }, svg);
  el('path', { d: 'M-8 -235.5 H8 M0 -243.5 V-227.5', stroke: '#6fcf97', 'stroke-width': .6 }, svg);
  rect(-55, -290.5, 110, 110, '', svg, { fill: 'none', stroke: '#6f7c88', 'stroke-dasharray': '3 3', 'stroke-width': .6 });
  R.camFlash = rect(-120, -292, 240, 120, '', svg, { fill: '#fff', opacity: 0 });
}

/* ───────────────────────── 패널 골격 ───────────────────────── */
function buildTimeline() {
  const tl = $('timeline'); tl.innerHTML = '';
  const groups = [['home', META.groups.home], ['take', META.groups.take], ['swing1', META.groups.swing1],
    ['inspect', META.groups.inspect], ['swing2', META.groups.swing2], ['out', META.groups.out], ['handoff', '인계']];
  R.tc = {};
  for (const [gk, gl] of groups) {
    const steps = META.steps.filter((s) => s.group === gk);
    const d = document.createElement('div'); d.className = 'tg' + (gk.startsWith('swing') ? ' swing' : '');
    d.title = gl; d.style.setProperty('--n', steps.length);
    const cs = document.createElement('div'); cs.className = 'cs'; d.appendChild(cs);
    for (const st of steps) {
      const c = document.createElement('div'); c.className = 'tc'; c.title = `[${st.key}] ${st.title}\n${st.why}`;
      c.textContent = st.key === 'HO' ? '인계' : st.key;
      const pg = document.createElement('span'); pg.className = 'pg'; pg.style.width = '0'; c.appendChild(pg);
      cs.appendChild(c); R.tc[st.key] = { c, pg };
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
  row('theta', 'θ1', -5, 185, '°', [0, 90, 180]);
  row('zc', 'ZC', GEO.zc_min, GEO.zc_max, 'mm', [GEO.zc_swing, 181, 192]);
  row('rx', 'RX', 0, GEO.rx_stroke_measured, 'mm', [GEO.rx_ext]);
  R.ax.rx.av.title = '지령값 · 전진 도달 센서 없음';
  const chips = $('chips'); chips.innerHTML = '';
  R.chip = {};
  for (const [k, lbl] of [['S1', 'θ1 원점'], ['S2', 'Z 원점'], ['S3', 'R 후진'], ['S4', '블레이드 웨이퍼'], ['S5', '반출 점유'], ['ESTOP', '비상정지']]) {
    const c = document.createElement('div'); c.className = 'chip' + (k === 'ESTOP' ? ' estop' : '');
    c.innerHTML = `<b>${k === 'ESTOP' ? 'E-STOP' : k}</b>`; c.dataset.lbl = lbl; c.title = lbl;
    chips.appendChild(c); R.chip[k] = c;
  }
  const sb = $('slotbar'); sb.innerHTML = '';
  R.sl = [];
  for (let k = 1; k <= GEO.slot_count; k++) { const d = document.createElement('div'); d.className = 'sl unknown'; d.textContent = String(k); sb.appendChild(d); R.sl.push(d); }
  const lgd = $('loadgrid'); lgd.innerHTML = '';
  for (let k = 1; k <= GEO.slot_count; k++) {
    const l = document.createElement('label'); l.innerHTML = `<input type="checkbox" data-slot="${k}" checked> 슬롯 ${String(k).padStart(2, '0')}`; lgd.appendChild(l);
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
  offset = offset === null ? o : Math.min(offset + 0.0005, o);  // 가장 빠른 도착 기준(천천히 따라감)
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
    if (c === 'cassette_ready' && j.result && j.result.need_manual) askManualN(j.result);
    return j;
  } catch (e) { toast('명령 전송 실패 (서버 응답 없음)'); return null; } finally { clearTimeout(tm); }
}
function handleCmdError(c, j) {
  if (j.error === 'need_c90_confirm') { confirmC90(); return; }
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
  for (const id of ['S2', 'S3', 'S4']) { const s = R.sen[id]; s.t.setAttribute('transform', `rotate(${p.theta} ${s.x} ${s.y})`); }
  // 입면도
  R.car.setAttribute('transform', `translate(0,${-(p.zc - GEO.zc_swing)})`);
  R.bladeE.setAttribute('transform', `translate(${p.rx},0)`);
  R.rodE.setAttribute('width', 58 + p.rx);
  const st = nearStation(p.theta);
  const dest = destStation();
  for (const k of ['cassette', 'inspect', 'output']) {
    const show = st === k || (!st && dest === k);
    R.st[k].style.display = show ? '' : 'none';
    R.st[k].classList.toggle('ghoststn', !st && dest === k);
  }
  // 카메라 개념도: 90° 부근이면 포크 그림자
  if (R.camTines) {
    const near90 = Math.abs(p.theta - 90) < 2;
    R.camTines.style.display = near90 ? '' : 'none';
    R.camTines.setAttribute('transform', `translate(0,${-p.rx})`);
    if (snap && snap.occ.blade && near90) { R.camW.setAttribute('cy', -(184.5 + p.rx)); R.camW.style.display = ''; }
  }
}
function destStation() {
  if (!snap || !snap.step) return null;
  const g = snap.step.group; const key = snap.step.key;
  if (g === 'swing1') return 'inspect';
  if (g === 'swing2') return 'output';
  if (key === '21') return 'cassette';
  if (g === 'home') return 'cassette';
  return snap.step.stage;
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
  // 완료 수(하단) + 상세 수치
  const c = s.cassette;
  $('st-done').innerHTML = c.ready ? `${s.counts.done}<span class="of"> / ${c.n_est}</span>` : `${s.counts.done}`;
  $('st-done').title = c.ready ? (c.manual ? '투입 수 수동 입력' : '투입 추정 (TOF)') : '';
  $('cnt-manual').hidden = !(c.ready && c.manual);
  $('cnt-mismatch').hidden = !c.mismatch;
  $('st-cycle').textContent = s.counts.cycle_avg ? `${fmt(s.counts.cycle_avg)} s` : '-';
  $('st-cycle').title = s.counts.cycle_last ? `마지막 ${fmt(s.counts.cycle_last)} s` : '';
  $('cnt-n').textContent = c.ready ? c.n_est : '-';
  $('cnt-conf').textContent = c.confirmed;
  $('cnt-rem').textContent = c.ready ? c.remaining : '-';
  $('cnt-tof').textContent = s.tof.d !== null ? `${fmt(s.tof.d)} mm` : '-';
  // 흐름 띠: 주황 '지금'은 로봇의 현재 단계 하나만, 인계 대기 웨이퍼는 라일락 점
  const stage = s.step ? s.step.stage : null;
  for (const li of document.querySelectorAll('#stages li')) {
    const k = li.dataset.stage;
    li.classList.toggle('on', k === stage);
    li.classList.toggle('par', k === 'handoff' && !!s.occ.C180);
  }
  for (const k of ['cassette', 'inspect', 'output']) R.hi[k].classList.toggle('on', k === stage);
  $('nar-now').textContent = s.step ? `[${s.step.key}] ${s.step.title}` : '';
  // 도면: 웨이퍼 존재
  R.casW.style.display = s.slots.some((x) => x.sim_present) ? '' : 'none';
  R.c90W.style.display = s.occ.C90 ? '' : 'none';
  R.c180W.style.display = s.occ.C180 ? '' : 'none';
  R.carry.style.display = s.occ.blade ? '' : 'none';
  R.carryE.style.display = s.occ.blade ? '' : 'none';
  R.c90Ew.style.display = s.occ.C90 ? '' : 'none';
  R.c180Ew.style.display = s.occ.C180 ? '' : 'none';
  s.slots.forEach((x, i) => {
    const r = R.slotW[i]; r.style.display = x.sim_present ? '' : 'none'; r.setAttribute('class', `wafer s-${x.status}`);
  });
  const col = s.alarm && s.alarm.code === 'COLLISION';
  R.collide.classList.toggle('on', !!col); R.collideE.classList.toggle('on', !!col);
  // 센서 LED(+ 알람이 지목한 센서는 빨간 링) + 칩
  const almSen = s.alarm ? SENSOR_OF_ALARM[s.alarm.code] : null;
  for (const k of ['S1', 'S2', 'S3', 'S4', 'S5']) {
    const on = !!s.sensors[k];
    R.sen[k].c.setAttribute('class', 'sen' + (on ? ' on' : '') + (s.expect && s.expect.sensor === k ? ' exp' : '') + (almSen === k ? ' alm' : ''));
  }
  R.s5E.setAttribute('class', 'sen' + (s.sensors.S5 ? ' on' : ''));
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
  // 축(상세): 지령값. 목표는 눈금으로만
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
  s.slots.forEach((x, i) => { const d = R.sl[i]; d.className = `sl ${x.status}`; d.title = SLOT_KR[x.status] || x.status; });
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
  // 캡처(실카메라 저장본만, 로그 서랍 안)
  const caps = $('caps'); const files = s.captures.filter((x) => x.file);
  caps.hidden = !files.length;
  caps.innerHTML = files.slice().reverse().map((x) => `<span class="cap">슬롯 ${x.slot} <a href="/captures/${encodeURIComponent(x.file)}" target="_blank">jpg</a></span>`).join('');
}
function esc(s) { return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])); }

// 알람 바 = 주의를 끄는 유일한 자리. 알람/운전자 확인/인계 지연(60s 경고 뒤)만. 키가 바뀔 때만 다시 그린다.
function renderAttention(s) {
  if (s.state !== 'WAIT_S5') s5Late = false;
  let key = null, text = '', cls = 'alarmbar', title = '';
  if (s.alarm) {
    const a = s.alarm; const msg = String(a.msg || a.code);
    text = msg + (a.step && !msg.includes('단계') ? ` · 단계 ${a.step}` : '') + (a.fatal ? ' — 원점 복귀 필요' : '');
    title = a.code; key = `A|${a.code}|${a.t}|${text}`;
  } else if (s.state === 'WAIT_OPERATOR' && s.prompt) {
    text = `슬롯 ${s.prompt.slot} 확인 필요`; cls += ' wait'; key = `P|${s.prompt.slot}`;
  } else if (s.state === 'WAIT_S5' && s5Late) {
    text = '인계 지연 — 다음 이송로봇 확인'; cls += ' wait'; key = 'S5';
  }
  if (key === attnKey) return;
  attnKey = key;
  const ab = $('alarmbar');
  if (!key) { ab.hidden = true; return; }
  ab.hidden = false; ab.className = cls; ab.textContent = text; ab.title = title;
}
function updateTimeline(s) {
  const key = s.step ? s.step.key : null;
  const order = META.steps.map((x) => x.key);
  const ci = key ? order.indexOf(key) : -1;
  const homing = key && key.startsWith('H');
  for (const k in R.tc) {
    const t = R.tc[k]; const i = order.indexOf(k);
    const isCur = k === key;
    let done = false;
    if (ci >= 0) done = homing ? (k.startsWith('H') && i < ci) : (!k.startsWith('H') && k !== 'HO' && i < ci) || (!homing && s.homed && k.startsWith('H'));
    t.c.classList.toggle('cur', isCur);
    t.c.classList.toggle('par', k === 'HO' && !!s.occ.C180);
    t.c.classList.toggle('done', done && !isCur);
    t.pg.style.width = isCur ? `${Math.round((s.step.action + 0.5) / s.step.actions * 100)}%` : (k === 'HO' && s.handoff_left !== null ? `${Math.min(100, Math.max(0, 100 - s.handoff_left / GEO.speeds.handoff * 100))}%` : '0');
  }
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
    if (!s.occ.blade) { R.camW.setAttribute('cy', -235.5); R.camW.style.display = s.occ.C90 ? '' : 'none'; }
    if (s.occ.blade && Math.abs(s.pose.theta - 90) >= 2) R.camW.style.display = 'none';
    const capturing = s.step && s.step.key === '11' && s.step.action === 1 && s.state === 'RUNNING';
    R.camLight.setAttribute('opacity', capturing ? 1 : 0.35);
    R.camFlash.setAttribute('opacity', capturing ? 0.12 : 0);
  }
}
// 헤더 버튼: 지금 누를 수 있는 것만 고정 순서로, 다음 동작은 강조. 소프트 정지는 항상 맨 끝 같은 자리.
function updateControls(s) {
  const dis = stale || BOOT.view_only;
  const st = s.state, c = s.cassette;
  const vis = {
    home: ALLOWED.home.includes(st),
    cassette_ready: ALLOWED.cassette_ready.includes(st),
    start: ALLOWED.start.includes(st) && c.ready && c.remaining > 0,
    resume: st === 'PAUSED',
    pause: ALLOWED.pause.includes(st),
    prompt: st === 'WAIT_OPERATOR' && !!s.prompt,
    reset: ALLOWED.reset.includes(st),
    c90: !s.c90_verified && ALLOWED.c90.includes(st),
    blade: (s.occ.blade || !!s.sensors.S4) && ALLOWED.blade.includes(st),
    stop: true,
  };
  let primary = null;
  if (st === 'IDLE') primary = 'home';
  else if (st === 'READY' || st === 'CASSETTE_DONE') primary = vis.blade ? 'blade' : vis.start ? 'start' : 'cassette_ready';
  else if (st === 'PAUSED') primary = 'resume';
  else if (st === 'ALARM' || st === 'ESTOP') primary = 'reset';
  else if (st === 'WAIT_OPERATOR') primary = 'prompt';
  for (const b of document.querySelectorAll('#controls [data-cmd]')) {
    const k = b.dataset.cmd;
    b.hidden = !vis[k];
    b.disabled = dis || (k === 'stop' && !ALLOWED.stop.includes(st));
    b.classList.toggle('primary', k === primary);
  }
  $('btn-estop').disabled = dis || st === 'ESTOP';
  for (const id of ['btn-load', 'btn-fault']) $(id).disabled = dis;
  for (const b of document.querySelectorAll('#speed button')) b.disabled = dis;
}

/* ───────────────────────── 로그 ───────────────────────── */
function addLog(items) {
  const ol = $('log');
  for (const e of items) {
    logRing.push(e);
    if (e.code === 'S5_WAIT' && snap && snap.state === 'WAIT_S5') s5Late = true;
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
  modal(`<h3>슬롯 ${p.slot}: 웨이퍼 감지 안 됨</h3><p>카세트를 눈으로 확인하세요.</p><div class="row"><button class="b" id="m-retry">재시도 (웨이퍼 있음)</button><button class="b go" id="m-empty">빈 슬롯으로 처리</button></div>`,
    (close) => {
      $('m-retry').addEventListener('click', () => { close(); cmd('confirm', { choice: 'retry' }, { expect: 'WAIT_OPERATOR' }); });
      $('m-empty').addEventListener('click', () => { close(); cmd('confirm', { choice: 'empty' }, { expect: 'WAIT_OPERATOR' }); });
    });
}
function confirmC90() {
  const sim = isSim();
  modal(`<h3>검사대 확인</h3><p>검사 크래들에 웨이퍼가 없는지 눈으로 확인하세요 (센서 없음).</p><div class="row"><button class="b" data-close>취소</button>${sim ? '<button class="b" id="m-rm">(시뮬) 웨이퍼 제거</button>' : ''}<button class="b go" id="m-clear">비어 있음 확인</button></div>`,
    (close) => {
      $('m-clear').addEventListener('click', () => { close(); cmd('confirm', { choice: 'c90_clear' }); });
      if (sim) $('m-rm').addEventListener('click', () => { cmd('remove_wafer', { loc: 'C90' }); });
    });
}
function confirmBlade(thenStart) {
  // 블레이드 위 웨이퍼: 운전자가 손으로 치운 것을 확인 -> 그 슬롯은 '제거'로 기록, 다음 슬롯부터 진행
  const w = snap && snap.wafers.find((x) => x.loc === 'blade');
  const sim = isSim();
  if (!w) {  // 웨이퍼 없이 S4 만 켜짐 = 센서 오감지
    modal(`<h3>S4 오감지</h3><p>블레이드에 웨이퍼가 없습니다.</p><div class="row"><button class="b" data-close>닫기</button>${sim ? '<button class="b go" id="m-f">오감지 끄기</button>' : ''}</div>`,
      (close) => { if (sim) $('m-f').addEventListener('click', () => { close(); cmd('fault', { name: 's4_false_present', on: false }); }); });
    return;
  }
  modal(`<h3>블레이드 위 웨이퍼 (슬롯 ${w.slot})</h3><p>손으로 치운 뒤 누르세요.</p><div class="row"><button class="b" data-close>취소</button><button class="b go" id="m-rm">제거 완료${thenStart ? ' → 시작' : ''}</button></div>`,
    (close) => {
      $('m-rm').addEventListener('click', async () => {
        close();
        const r = await cmd('remove_wafer', { loc: 'blade' });
        if (r && r.ok && thenStart && snap && ['READY', 'CASSETTE_DONE'].includes(snap.state)) cmd('start', {}, { expect: snap.state });
      });
    });
}
function askManualN(r) {
  modal(`<h3>투입 개수 입력</h3><p>TOF 로 판정 불가 (${fmt(r.d)} mm)</p><div class="row"><input type="number" id="m-n" min="0" max="10" value="1"><span class="sp"></span><button class="b" data-close>취소</button><button class="b go" id="m-ok">확정</button></div>`,
    (close) => { $('m-ok').addEventListener('click', () => { const n = Number($('m-n').value); close(); cmd('cassette_ready', { manual_n: n }); }); });
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
      if (c === 'c90') return confirmC90();
      if (c === 'blade') return confirmBlade(false);
      if (c === 'prompt') return snap && snap.prompt && showPrompt(snap.prompt);
      if (c === 'estop') closePops();
      if (b.dataset.confirm) return confirmCmd(c);
      cmd(c, {}, { expect: snap && snap.state });
    });
  }
  for (const b of document.querySelectorAll('#speed button')) b.addEventListener('click', () => cmd('speed', { value: Number(b.dataset.speed) }));
  const pop = (id, btn) => $(btn).addEventListener('click', () => { const p = $(id); const open = p.hidden; closePops(); p.hidden = !open; });
  pop('pop-tools', 'btn-tools'); pop('pop-load', 'btn-load'); pop('pop-fault', 'btn-fault');
  for (const b of document.querySelectorAll('#pop-load [data-fill]')) b.addEventListener('click', () => {
    const f = b.dataset.fill; const boxes = document.querySelectorAll('#loadgrid input');
    boxes.forEach((x, i) => { x.checked = f === 'gap' ? [0, 1, 3, 4, 5].includes(i) : i < Number(f); });
  });
  $('load-apply').addEventListener('click', () => {
    const slots = [...document.querySelectorAll('#loadgrid input')].map((x) => x.checked);
    cmd('load_map', { slots }); $('pop-load').hidden = true;
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
  // 상세: URL > 저장값 > 기본(시뮬 OFF, 실장비 ON)
  const qd = q.get('detail'), sd = store('wc-detail');
  setDetail(qd !== null ? qd === '1' : sd !== null ? sd === '1' : META.source !== 'SIM', false);
  buildPlan(); buildElev(); buildCam(); buildTimeline(); buildPanels(); bindUi();
  connect();
  setInterval(watchdog, 250);
  requestAnimationFrame(frame);
}
init();
