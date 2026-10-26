/* R22 웨이퍼 검사셀 3D 보기 (보기 전용) — 2D 화면과 같은 SSE 스냅샷(SIM/LIVE)으로 움직인다.
   좌표: three Y = 셀 z(mm), three (X, Z) = 2D 평면도 SVG 셀 좌표(x, -y). 그래서 2D 의 rotate(-θ) = three rotation.y = +θ,
   스테이션 로컬 좌표(반경 u, 접선 v)도 2D 와 같은 숫자를 그대로 쓴다. 모양은 상자/원기둥 근사(CAD 아님).
   책상 · 대차 몸체 치수는 cell_config 의 가정값. 조작 명령은 보내지 않는다.
*/
import * as THREE from 'three';
import { OrbitControls } from './OrbitControls.js';

const $ = (id) => document.getElementById(id);
const D2R = Math.PI / 180;
const STATE_KR = {
  IDLE: '원점 필요', HOMING: '원점 복귀 중', READY: '대기', RUNNING: '공정 진행',
  WAIT_CART: '대차 대기', WAIT_NG: '불량 카세트 교체', PAUSED: '일시정지', ALARM: '알람', ESTOP: '비상정지',
};
const LINK_KR = { OK: '장비 연결', DOWN: '연결 끊김', WAIT_HELLO: '응답 대기', STALE: '데이터 멈춤', VERSION: '프로토콜 불일치', NO_PYSERIAL: 'pyserial 없음' };
const PALETTE = {
  dark: { bg: 0x0d1115, floor: 0x10161b, grid1: 0x2a3540, grid2: 0x1b232b, desk: 0x2b3642, deskEdge: 0x7f93a6, leg: 0x222b34,
    part: 0x3e4b58, part2: 0x56677a, glass: 0x8fa3b8, steel: 0x8fb4d8, steel2: 0x3d5a78, rob: 0x24384d, wafer: 0xb9b3ee,
    ok: 0x6fcf97, ng: 0xff7a66, amber: 0xf0a93b, uv: 0xb58cff, cart: 0x34424f, wheel: 0x1c242b, led: 0x2d3943, back: 0xe9b35f },
  light: { bg: 0xe9ecea, floor: 0xdde2df, grid1: 0xaab4bc, grid2: 0xc9d0d5, desk: 0xd7cbb0, deskEdge: 0x5d6b78, leg: 0x9aa4ad,
    part: 0xaab6c0, part2: 0x8e9caa, glass: 0x7f8f9e, steel: 0x2e5580, steel2: 0x7f9bb8, rob: 0x9db4cc, wafer: 0x5f64a8,
    ok: 0x2b7050, ng: 0xb0402f, amber: 0xb16a04, uv: 0x6d3fc4, cart: 0x8796a3, wheel: 0x4b5560, led: 0xc4ccd3, back: 0xd99a3a },
};

let GEO = null, META = null, snap = null, theme = 'dark';
const buf = [];
let offset = null, lastTick = -1, lastTickAt = 0, lastMsgAt = 0, follow = false;
const R = {};                 // 움직이는 부품 참조
const labels = [];

/* ───────────────────────── 장면 도우미 ───────────────────────── */
// 재질은 (팔레트 키 + 옵션)마다 하나만 만들고, 테마를 바꾸면 색만 바꾼다
function mat(key, opts) {
  const m = new THREE.MeshStandardMaterial(Object.assign({ roughness: 0.75, metalness: 0.05 }, opts || {}));
  m.userData.key = key;
  const c = PALETTE[theme][key];
  if (c !== undefined) m.color.setHex(c);
  return m;
}
const MATS = [];
function useMat(key, opts) {
  const id = key + JSON.stringify(opts || {});
  let m = MATS.find((x) => x.userData.id === id);
  if (!m) { m = mat(key, opts); m.userData.id = id; MATS.push(m); }
  return m;
}
// 2D 로컬 (x, y_svg) + 높이 z0..z1 상자
function box(parent, x0, x1, y0, y1, z0, z1, m) {
  const g = new THREE.BoxGeometry(Math.max(0.5, x1 - x0), Math.max(0.5, z1 - z0), Math.max(0.5, y1 - y0));
  const mesh = new THREE.Mesh(g, m);
  mesh.position.set((x0 + x1) / 2, (z0 + z1) / 2, (y0 + y1) / 2);
  parent.add(mesh);
  return mesh;
}
function cyl(parent, cx, cy, r, z0, z1, m, seg) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, Math.max(0.5, z1 - z0), seg || 40), m);
  mesh.position.set(cx, (z0 + z1) / 2, cy);
  parent.add(mesh);
  return mesh;
}
function group(parent) { const g = new THREE.Group(); if (parent) parent.add(g); return g; }
function edges(mesh, color) {
  const l = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry), new THREE.LineBasicMaterial({ color }));
  l.position.copy(mesh.position); l.rotation.copy(mesh.rotation); mesh.parent.add(l);
  return l;
}

/* ───────────────────────── 장면 만들기 ───────────────────────── */
let renderer, scene, camera, controls, hemi, sun;
function buildScene() {
  const canvas = $('c3d');
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(38, 1, 5, 30000);
  controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.target.set(0, 60, -60);
  hemi = new THREE.HemisphereLight(0xffffff, 0x445566, 1.4);
  scene.add(hemi);
  sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(900, 2200, 1400);
  scene.add(sun);
  const cell = group(scene);
  const dk = GEO.desk;
  // 바닥 · 책상(윗면 + 테두리 + 다리)
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(6000, 6000), useMat('floor'));
  floor.rotation.x = -Math.PI / 2; floor.position.y = dk.floor_z; cell.add(floor);
  R.cell = cell;
  makeGrid();
  // 셀 y -> SVG y = -y
  const top = box(cell, dk.x0, dk.x1, -dk.y1, -dk.y0, dk.top_z - 25, dk.top_z, useMat('desk'));
  R.deskEdge = edges(top, PALETTE.dark.deskEdge);
  for (const [x, y] of [[dk.x0 + 30, -dk.y1 + 30], [dk.x1 - 30, -dk.y1 + 30], [dk.x0 + 30, -dk.y0 - 30], [dk.x1 - 30, -dk.y0 - 30]]) {
    box(cell, x - 20, x + 20, y - 20, y + 20, dk.floor_z, dk.top_z - 25, useMat('leg'));
  }
  // 고정부: 받침, θ 모터, TS-11 링 + θ 자리 센서 5
  cyl(cell, 0, 0, 85, dk.top_z, 85, useMat('part2'));
  box(cell, 37.5, 79.8, -21.2, 21.2, dk.top_z, 85, useMat('part'));
  const ring = new THREE.Mesh(new THREE.LatheGeometry([new THREE.Vector2(54, -24), new THREE.Vector2(73, -24), new THREE.Vector2(73, -20), new THREE.Vector2(54, -20), new THREE.Vector2(54, -24)], 64), useMat('part'));
  cell.add(ring);
  R.ths = {};
  GEO.station_order.forEach((k, i) => {
    const a = GEO.stations[k] * D2R;
    const g = group(cell); g.rotation.y = a;
    box(g, 44, 66, -6, 6, -42, -33, useMat('leg'));
    const led = new THREE.Mesh(new THREE.SphereGeometry(7, 16, 12), new THREE.MeshStandardMaterial({ color: PALETTE.dark.led, emissive: 0x000000 }));
    led.position.set(76, -22, 0); g.add(led);
    R.ths[GEO.th_sensors[i]] = led;
  });
  // 장소
  R.cass = {};
  for (const k of GEO.cassettes) {
    const g = group(cell); g.rotation.y = GEO.stations[k] * D2R;
    const dx = GEO.station_r[k] - 235.5;
    box(g, 280 + dx, 292 + dx, -62, -50, dk.top_z, 172, useMat('part'));       // 도크 기둥
    box(g, 280 + dx, 292 + dx, 50, 62, dk.top_z, 172, useMat('part'));
    box(g, 236.5 + dx, 250.5 + dx, -7, 7, 134, 140, useMat('steel2'));          // ToF 칩(밑에서 위를 봄)
    const c = makeCassette(k === 'ng' ? 'ng' : 'in');
    c.g.position.set(dx, 172, 0); g.add(c.g);
    if (k === 'input') {
      R.clamp = box(c.g, 205, 305, -70, 70, GEO.cass_plate_under, GEO.cass_plate_under + 6, useMat('amber', { transparent: true, opacity: 0.18, depthWrite: false }));
    }
    R.cass[k] = c;
  }
  // ② 검사 크래들 + 백라이트 + StreamCam
  const ig = group(cell); ig.rotation.y = GEO.stations.inspect * D2R;
  box(ig, 276, 296, -10, 10, dk.top_z, 244, useMat('part'));
  box(ig, 262, 318, -52, 52, 244, 250, useMat('part2'));
  box(ig, 220, 265, -52, -40, 244, 250, useMat('part'));
  box(ig, 220, 265, 40, 52, 244, 250, useMat('part'));
  for (const [x, y] of [[222, -43], [222, 43], [280.5, 0]]) cyl(ig, x, y, 4, 250, GEO.cradle_seat_z, useMat('leg'));
  R.backlight = box(ig, 238, 268, -50, 50, 213, 240, useMat('back', { emissive: PALETTE.dark.back, emissiveIntensity: 0.25 }));
  R.inspW = cyl(ig, 235.5, 0, 50, GEO.cradle_seat_z, GEO.cradle_seat_z + GEO.wafer_t, useMat('wafer'), 48);
  box(ig, 300, 310, -5, 5, 250, 510, useMat('part'));
  box(ig, 225, 310, -6, 6, 500, 510, useMat('part'));
  box(ig, 222, 249, -30, 30, 470, 500, useMat('rob'));                           // 카메라(위에서 찍음)
  // UV 탑: 기둥 · 받침 패드 · 웨이퍼 · 빛 · 랜턴
  const ug = group(cell); ug.rotation.y = GEO.stations.uv * D2R;
  const UR = GEO.station_r.uv, sz = GEO.uv_seat_z;
  box(ug, UR + 58, UR + 70, -10, 10, dk.top_z, 265, useMat('part'));
  box(ug, UR, UR + 70, -8, 8, 257, 265, useMat('part'));
  for (const [x, y] of [[UR - 13.5, -43], [UR - 13.5, 43], [UR + 45, 0]]) cyl(ug, x, y, 4, sz - 4, sz, useMat('leg'));
  box(ug, UR + 40, UR + 58, -50, 50, sz - 6, sz - 4, useMat('part'));
  R.uvW = cyl(ug, UR, 0, 50, sz, sz + GEO.wafer_t, useMat('ok'), 48);
  R.uvBeam = cyl(ug, UR, 0, 46, sz + 3, 217, new THREE.MeshBasicMaterial({ color: PALETTE.dark.uv, transparent: true, opacity: 0.35, depthWrite: false }), 40);
  R.uvLamp = cyl(ug, UR, 0, 52, 217, 257, new THREE.MeshStandardMaterial({ color: PALETTE.dark.part2, emissive: 0x000000 }), 40);
  // 대차(바닥, 받침 윗면 = 셀 높이)
  buildCart(cell);
  // 로봇(회전부 -> Z 캐리지 -> R 포크)
  buildRobot(cell);
  // 이름표(HTML, 매 프레임 투영)
  const lab = (text, k, r, z, cls) => {
    const a = (typeof k === 'number' ? k : GEO.stations[k]) * D2R;
    addLabel(text, new THREE.Vector3(r * Math.cos(a), z, -r * Math.sin(a)), cls);
  };
  lab('① 투입', 'input', 300, 330); lab('② 검사', 'inspect', 290, 330); lab('③ 불량', 'ng', 290, 330);
  lab('UV 탑', 'uv', UR, 300); lab('옆길', GEO.side_theta, 230, 40, 'small');
  addLabel('책상', new THREE.Vector3(dk.x1 - 60, dk.top_z + 4, -dk.y0 - 30), 'small');     // UV · 대차 쪽 모서리(대차에 안 가림)
  R.cartLabelAnchor = new THREE.Vector3(0, 0, 0);
  R.cartLabel = addLabel('대차', R.cartLabelAnchor, '');
}

function makeGrid() {
  const P = PALETTE[theme];
  if (R.grid) { R.cell.remove(R.grid); R.grid.geometry.dispose(); R.grid.material.dispose(); }
  R.grid = new THREE.GridHelper(6000, 60, P.grid1, P.grid2);        // 격자 색은 만들 때 정해짐 -> 테마마다 다시 만듦
  R.grid.position.y = GEO.desk.floor_z + 1; R.cell.add(R.grid);
}

function makeCassette(kind) {
  // R9 카세트 단면을 3D 로: 로컬 x 215~300, 웨이퍼 중심 235.5, 바닥 z = 0 (도크에서는 172)
  const g = group(null);
  const pm = useMat('glass', { transparent: true, opacity: 0.32, depthWrite: false });
  box(g, 215, 300, -62, 62, 0, 8, useMat('part'));                              // 바닥판
  box(g, 215, 294, -62, -57, 8, 116, pm); box(g, 215, 294, 57, 62, 8, 116, pm); // 옆벽
  box(g, 288, 294, -57, 57, 8, 116, pm);                                          // 뒷벽
  box(g, 215, 292, -62, -47, 112, 116, useMat('part')); box(g, 215, 292, 47, 62, 112, 116, useMat('part'));   // 윗레일
  const wafers = [];
  for (let n = 1; n <= GEO.slot_count; n++) {
    const sz = GEO.slot_z0 + GEO.slot_pitch * (n - 1) - 172;
    box(g, 215, 286, -57, -47, sz - 4, sz, useMat('part')); box(g, 215, 286, 47, 57, sz - 4, sz, useMat('part'));
    wafers[n] = cyl(g, 235.5, 0, 50, sz, sz + GEO.wafer_t, useMat(kind === 'ng' ? 'ng' : 'wafer'), 48);
  }
  return { g, wafers };
}

function buildCart(cell) {
  const dk = GEO.desk;
  const outer = group(cell), inner = group(outer);
  inner.position.set(-100, 0, -130);
  R.cart = outer;
  const [bx, by, bw, bh] = GEO.cart_body;
  const top = GEO.cart_seat_top_z;
  // 바닥 섀시 + 모서리 기둥 4 + 상판(받침 윗면 = 셀 높이). 속이 비어 책상 앞을 가리지 않게
  R.cartBody = box(inner, bx, bx + bw, by, by + bh, dk.floor_z + 60, dk.floor_z + 240, useMat('cart'));
  R.cartParts = [R.cartBody];
  for (const [x, y] of [[bx + 25, by + 25], [bx + bw - 25, by + 25], [bx + 25, by + bh - 25], [bx + bw - 25, by + bh - 25]]) {
    R.cartParts.push(box(inner, x - 18, x + 18, y - 18, y + 18, dk.floor_z + 240, top - 30, useMat('cart')));
  }
  R.cartParts.push(box(inner, bx, bx + bw, by, by + bh, top - 42, top - 30, useMat('cart')));
  for (const [x, y] of [[bx + 20, by + 40], [bx + bw - 20, by + 40], [bx + 20, by + bh - 40], [bx + bw - 20, by + bh - 40]]) {
    const w = cyl(inner, x, y, 40, 0, 30, useMat('wheel'), 24);
    w.rotation.z = Math.PI / 2; w.position.set(x, dk.floor_z + 40, y);
  }
  R.seat = {};
  for (const [s, [x, y]] of Object.entries(GEO.cart_seat_xy)) {
    box(inner, x - 62, x + 62, y - 62, y + 62, top - 30, top, useMat('part2'));
    // 카세트 입구(로컬 낮은 x)를 그리퍼 쪽으로: 앞 받침은 셀 쪽(-z), 뒤 받침은 바깥(+z) -> 대차가 180° 돌면 셀 쪽
    const sg = group(inner); sg.position.set(x, top, y);
    sg.rotation.y = (GEO.cart_seat_face[s] < 0 ? -1 : 1) * Math.PI / 2;     // +x(카세트 뒤) -> +z / -z
    const c = makeCassette('in');
    c.g.position.set(-235.5, 0, 0); sg.add(c.g);
    box(c.g, 205, 305, -70, 70, GEO.cass_plate_under, GEO.cass_plate_under + 6, useMat('amber', { transparent: true, opacity: 0.18, depthWrite: false }));   // 클램프 윗판째로 실림
    R.seat[s] = c;
  }
}

function buildRobot(cell) {
  const rot = group(cell); R.rot = rot;
  cyl(rot, 0, 0, 60, 85, 95, useMat('rob'));
  const notch = new THREE.Mesh(new THREE.CylinderGeometry(51, 51, 2, 48), useMat('leg'));
  notch.position.y = -37.5; rot.add(notch);
  box(rot, 46.9, 56, -2, 2, -39, -36, useMat('ng'));                                          // 노치(θ0 = 포크 방향)
  for (const [x, y] of [[31.82, -31.82], [-31.82, -31.82], [31.82, 31.82], [-31.82, 31.82]]) cyl(rot, x, y, 5, 95, 364, useMat('steel2'), 16);
  box(rot, -70, 70, -50, 50, 364, 374, useMat('rob'));
  box(rot, -61.15, -18.85, -21.15, 21.15, 374, 422, useMat('steel2'));
  const car = group(rot); R.car = car;            // ZC 105 기준으로 그리고 position.y = zc - 105
  box(car, -50, 50, -50, 50, 105, 135, useMat('rob'));
  box(car, -10, 38, -17, 17, 135, 155, useMat('steel2'));
  const rodG = new THREE.BoxGeometry(58, 14, 18); rodG.translate(29, 0, 0);
  R.rod = new THREE.Mesh(rodG, useMat('rob')); R.rod.position.set(38, 145, 0); car.add(R.rod);
  const fork = group(car); R.fork = fork;
  box(fork, 96, 107, -6, 6, 139, 151, useMat('steel2'));
  box(fork, 107, 141, -20, 20, 151, 165, useMat('steel2'));
  const pts = [[107, -30], [131, -30], [131, -36], [149, -36], [149, -34], [203, -34], [203, -20], [149, -20], [149, 20], [203, 20], [203, 34], [149, 34], [149, 36], [131, 36], [131, 30], [107, 30]];
  const shape = new THREE.Shape(pts.map(([x, y]) => new THREE.Vector2(x, y)));
  const bg = new THREE.ExtrudeGeometry(shape, { depth: 4, bevelEnabled: false });
  bg.rotateX(Math.PI / 2);                         // (x, y, 0..4) -> (x, -(0..4), y)
  const blade = new THREE.Mesh(bg, useMat('steel')); blade.position.y = 169; fork.add(blade);
  for (const x of [149, 193]) for (const y of [-27, 27]) box(fork, x, x + 8, y - 4, y + 4, 169, 170.5, useMat('leg'));
  R.carry = cyl(fork, 184.5, 0, 50, 170.5, 170.5 + GEO.wafer_t, useMat('wafer'), 48);
  const cc = makeCassette('in');
  cc.g.position.set(GEO.station_r.input - 235.5 - GEO.station_rx.input, 170.5 - GEO.cass_plate_under, 0);
  fork.add(cc.g); R.carryCass = cc;
  box(cc.g, 205, 305, -70, 70, GEO.cass_plate_under, GEO.cass_plate_under + 6, useMat('amber', { transparent: true, opacity: 0.18, depthWrite: false }));
  R.s4 = new THREE.Mesh(new THREE.SphereGeometry(6, 16, 12), new THREE.MeshStandardMaterial({ color: PALETTE.dark.led }));
  R.s4.position.set(GEO.s4_local[0], 172, GEO.s4_local[1]); fork.add(R.s4);
  R.collide = new THREE.Mesh(new THREE.SphereGeometry(18, 16, 12), new THREE.MeshBasicMaterial({ color: 0xff3b30, transparent: true, opacity: 0.7 }));
  R.collide.position.set(203, 167, 0); fork.add(R.collide);
}

function addLabel(text, pos, cls) {
  const d = document.createElement('div'); d.className = 'lbl3' + (cls ? ' ' + cls : ''); d.textContent = text;
  $('labels').appendChild(d);
  const L = { d, pos }; labels.push(L);
  return L;
}

/* ───────────────────────── 테마 ───────────────────────── */
function applyTheme(t) {
  theme = t; document.body.dataset.theme = t;
  const P = PALETTE[t];
  scene.background = new THREE.Color(P.bg);
  for (const m of MATS) if (P[m.userData.key] !== undefined) {
    m.color.setHex(P[m.userData.key]);
    if (m.emissive && m.userData.key === 'back') m.emissive.setHex(P.back);
  }
  makeGrid();
  R.deskEdge.material.color.setHex(P.deskEdge);
  R.uvBeam.material.color.setHex(P.uv);
  hemi.intensity = t === 'dark' ? 1.4 : 1.1;
  if (snap) updateState(snap);
}

/* ───────────────────────── 스냅샷 반영 ───────────────────────── */
const show = (o, on) => { o.visible = !!on; };
function wColor(j) { return j === 'NG' ? 'ng' : j === 'OK' ? 'ok' : 'wafer'; }
function setMat(mesh, key) { const m = useMat(key); if (mesh.material !== m) mesh.material = m; }
function updateState(s) {
  const P = PALETTE[theme];
  const live = s.source !== 'SIM';
  const sb = $('src-badge'); sb.textContent = live ? '실장비' : '시뮬레이션'; sb.className = 'srcbadge' + (live ? ' live' : '');
  const lb = $('link-badge'); const ls = (s.link || {}).status;
  lb.hidden = !live;
  if (live) {
    const age = s.link.age_s !== null && s.link.age_s !== undefined ? Math.round(s.link.age_s * 1000) : null;
    lb.textContent = (LINK_KR[ls] || ls) + (ls === 'OK' && age !== null ? ` · ${age} ms` : '');
    lb.className = 'linkbadge ' + (ls === 'OK' ? 'ok' : ls === 'WAIT_HELLO' ? 'warn' : 'crit');
  }
  const stEl = $('st-state'); stEl.textContent = STATE_KR[s.state] || s.state; stEl.className = 's-' + s.state;
  $('headline').textContent = s.headline || '';
  // 알람 줄
  const ab = $('alarm3');
  let txt = null, cls = 'alarm3';
  if (live && ls !== 'OK') txt = `장비 ${LINK_KR[ls] || ls} - 데이터를 받지 못함`;
  else if (s.alarm) txt = s.alarm.msg + (s.alarm.fatal ? ' — 원점 복귀 필요' : '');
  else if (s.state === 'PAUSED' && s.pause_reason === 'LINK') { txt = s.headline; cls += ' wait'; }
  else if (s.state === 'WAIT_NG') { txt = '③ 불량 카세트 교체 필요'; cls += ' wait'; }
  ab.hidden = !txt; if (txt) { ab.textContent = txt; ab.className = cls; }
  // ① · ③ 카세트
  const di = s.docks.input, dn = s.docks.ng;
  show(R.cass.input.g, di.present); show(R.clamp, di.present);
  di.slots.forEach((x) => { const w = R.cass.input.wafers[x.n]; show(w, x.code === 'U' || x.code === 'K'); setMat(w, x.code === 'K' ? 'ok' : 'wafer'); });
  show(R.cass.ng.g, dn.present);
  dn.slots.forEach((x) => show(R.cass.ng.wafers[x.n], x.code === 'N'));
  // ② · UV
  show(R.inspW, s.inspect.wafer); setMat(R.inspW, wColor(s.inspect.judgment));
  show(R.uvW, s.uvh.wafer);
  const lamp = !!s.uv.lamp;
  show(R.uvBeam, lamp);
  R.uvLamp.material.emissive.setHex(lamp ? P.uv : 0x000000); R.uvLamp.material.emissiveIntensity = lamp ? 0.9 : 0;
  R.uvLamp.material.color.setHex(lamp ? P.uv : P.part2);
  const capturing = s.step && s.step.key === 'I3' && s.state === 'RUNNING';
  R.backlight.material.emissiveIntensity = capturing ? 1.0 : 0.25;
  // 포크
  show(R.carry, s.fork.wafer); setMat(R.carry, wColor(s.fork.judgment));
  show(R.carryCass.g, s.fork.cassette);
  // 센서
  for (const k in R.ths) {
    const on = !!s.sensors[k]; const bad = s.th_low >= 2;
    R.ths[k].material.color.setHex(bad ? P.ng : on ? P.ok : P.led);
    R.ths[k].material.emissive.setHex(bad ? P.ng : on ? P.ok : 0x000000); R.ths[k].material.emissiveIntensity = on || bad ? 0.8 : 0;
  }
  R.s4.material.color.setHex(s.sensors.S4 ? P.ok : P.led);
  R.s4.material.emissive.setHex(s.sensors.S4 ? P.ok : 0x000000);
  show(R.collide, s.alarm && s.alarm.code === 'COLLISION');
  // 대차
  const cart = s.cart;
  show(R.cart, cart.state !== 'away');
  R.cart.position.set(cart.lat + 100, 0, GEO.station_r.cart + cart.dist + 130);
  R.cart.rotation.y = -cart.rot * D2R;
  const ghost = cart.pos_known === false;                 // LIVE: 움직이는 동안 위치 데이터 없음 -> 반투명
  for (const m of R.cartParts) m.material = useMat('cart', ghost ? { transparent: true, opacity: 0.4 } : undefined);
  for (const [seat, code] of Object.entries(cart.seats)) {
    const c = R.seat[seat];
    show(c.g, code === 'F' || code === 'D');
    for (let n = 1; n <= GEO.slot_count; n++) setMat(c.wafers[n], code === 'D' ? 'ok' : 'wafer');
  }
  R.cart.updateMatrixWorld(true);
  R.cartLabelAnchor.set(100, GEO.cart_seat_top_z - 120, 130).applyMatrix4(R.cart.children[0].matrixWorld);
  R.cartLabel.d.style.display = cart.state === 'away' ? 'none' : '';
}

/* ───────────────────────── 통신 · 보간 ───────────────────────── */
function connect() {
  const es = new EventSource('/events');
  es.onmessage = (ev) => {
    let s; try { s = JSON.parse(ev.data); } catch (e) { return; }
    const now = performance.now() / 1000;
    lastMsgAt = now;
    const fresh = s.tick !== lastTick;
    if (fresh) { lastTick = s.tick; lastTickAt = now; buf.push(s); while (buf.length > 8) buf.shift(); }
    const o = s.t_src - Date.now() / 1000;
    offset = offset === null ? o : Math.min(offset + 0.0005, o);
    snap = s; window.snap3 = s;                 // 점검 스크립트용
    updateState(s);
  };
  es.onerror = () => { if (es.readyState === 2) setTimeout(connect, 1000); };
}
function interp() {
  if (!buf.length) return null;
  const lag = buf[buf.length - 1].source === 'SIM' ? 0.12 : 0.16;
  const tNow = Date.now() / 1000 + (offset || 0) - lag;
  let a = buf[0], b = buf[buf.length - 1];
  if (tNow >= b.t_src) return b.pose;
  if (tNow <= a.t_src) return a.pose;
  for (let i = 0; i < buf.length - 1; i++) { if (buf[i].t_src <= tNow && tNow <= buf[i + 1].t_src) { a = buf[i]; b = buf[i + 1]; break; } }
  const f = (tNow - a.t_src) / Math.max(1e-6, b.t_src - a.t_src);
  return { theta: a.pose.theta + (b.pose.theta - a.pose.theta) * f, zc: a.pose.zc + (b.pose.zc - a.pose.zc) * f, rx: a.pose.rx + (b.pose.rx - a.pose.rx) * f };
}

/* ───────────────────────── 카메라 · 그리기 ───────────────────────── */
// 시점(target 기준 오프셋). '위' 는 2D 평면도와 같은 방향(화면 위 = θ0 ③, 오른쪽 = 대차): 카메라를 -x 쪽으로 아주 조금 비킴
// 'UV 쪽' 은 UV 탑 자리(GEO.stations.uv) 바깥에서 본다 - 장소를 옮겨도 따라감
const VIEWS = {
  iso: [1000, 1350, 1450], top: [-2, 2350, 0], front: [0, 1150, 1600],
  side: () => { const a = GEO.stations.uv * D2R; return [1850 * Math.cos(a), 700, -1850 * Math.sin(a)]; },
};
function setView(k) {
  follow = false; $('btn-follow').classList.remove('on');
  const v = typeof VIEWS[k] === 'function' ? VIEWS[k]() : VIEWS[k]; controls.target.set(0, 60, -60);
  camera.position.set(v[0], v[1] + 60, v[2] - 60); controls.update();
}
function resize() {
  const w = window.innerWidth, h = window.innerHeight;
  renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
}
const tmp = new THREE.Vector3();
function frame() {
  const p = interp();
  if (p) {
    R.rot.rotation.y = p.theta * D2R;
    R.car.position.y = p.zc - GEO.zc_swing;
    R.fork.position.x = p.rx;
    R.rod.scale.x = Math.max(0.05, (58 + p.rx) / 58);
    if (follow) {
      const a = p.theta * D2R;
      tmp.set(-900 * Math.cos(a), 700, 900 * Math.sin(a));
      camera.position.lerp(tmp, 0.08); controls.target.lerp(new THREE.Vector3(0, 150, 0), 0.08);
    }
  }
  if (R.collide.visible) R.collide.material.opacity = 0.35 + 0.35 * Math.abs(Math.sin(performance.now() / 180));
  controls.update();
  renderer.render(scene, camera);
  const w = window.innerWidth, h = window.innerHeight;
  for (const L of labels) {
    tmp.copy(L.pos).project(camera);
    const vis = tmp.z < 1 && tmp.x > -1.05 && tmp.x < 1.05 && tmp.y > -1.05 && tmp.y < 1.05;
    if (L.d.style.display === 'none') continue;
    L.d.style.visibility = vis ? 'visible' : 'hidden';
    L.d.style.left = `${(tmp.x + 1) / 2 * w}px`; L.d.style.top = `${(1 - tmp.y) / 2 * h}px`;
  }
  requestAnimationFrame(frame);
}
function watchdog() {
  const now = performance.now() / 1000;
  let why = null;
  if (!lastMsgAt) why = '서버 연결 중…';
  else if (now - lastMsgAt > 1.5) why = '서버 연결 끊김';
  else if (now - lastTickAt > 1.5) why = snap && snap.source !== 'SIM' ? '장비 데이터 멈춤' : '시뮬레이터 멈춤';
  $('stale').hidden = !why; if (why) $('stale-why').textContent = why;
}

async function init() {
  let t = null;
  try { t = localStorage.getItem('wc-theme'); } catch (e) { t = null; }
  const q = new URLSearchParams(location.search);
  t = q.get('theme') || t || 'dark';
  try { META = await (await fetch('/api/meta')).json(); } catch (e) { $('stale').hidden = false; $('stale-why').textContent = '서버에 연결할 수 없음'; return; }
  GEO = META.geometry;
  document.title = `${META.cell} 3D 보기`;
  $('cellname').textContent = `웨이퍼 검사셀 ${META.cell} · 3D`;
  buildScene();
  applyTheme(t === 'light' ? 'light' : 'dark');
  resize(); window.addEventListener('resize', resize);
  setView(q.get('view') && VIEWS[q.get('view')] ? q.get('view') : 'iso');
  for (const b of document.querySelectorAll('#views [data-view]')) b.addEventListener('click', () => setView(b.dataset.view));
  $('btn-follow').addEventListener('click', () => { follow = !follow; $('btn-follow').classList.toggle('on', follow); });
  $('btn-theme').addEventListener('click', () => {
    const n = theme === 'dark' ? 'light' : 'dark'; applyTheme(n);
    try { localStorage.setItem('wc-theme', n); } catch (e) { /* 저장 못 해도 화면은 바뀜 */ }
  });
  window.R3 = R; window.cam3 = camera;          // 점검 스크립트용
  connect();
  setInterval(watchdog, 250);
  requestAnimationFrame(frame);
}
init();
