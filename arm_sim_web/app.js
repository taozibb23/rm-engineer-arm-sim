'use strict';
/* diesel 实物三自由度机械臂网页模拟器（合成/仿真级，非实机）。
 * 渲染风格：纯 SVG 手写 3D 投影，零依赖（无 Three.js）。
 * 数学来自 arm_kinematics.js（已对齐 C++）。本文件只做渲染/交互/演示。
 */
const K = window.ArmKin;
const $ = id => document.getElementById(id);
const DEG = Math.PI / 180;
const deg = r => r / DEG;
const fmt = (v, d = 3) => v.map(x => Number(x).toFixed(d)).join(', ');

// ---- 场景状态 ----
const L1 = K.L1, L2 = K.L2;
const GRASP_DROP = 0.030; // 夹住时物体中心在末端轴点下方此距离（教学假设，非实测）
// 手眼外参 T_tool_camera（占位，与 arm_sitl/config.json 一致：光轴朝下）
const T_tool_camera = K.se3(K.rotRpy([Math.PI, 0, 0]), [0, 0, 0]);
const state = {
  angles: { psi: 0, alpha: 0.5, beta: 0.5 },   // rad
  object: { pos: [0.18, 0.10, 0.022], size: 0.020, yaw: 0 },  // 头部样件（教学对象，非真实识别）
  install: { pos: [0.18, -0.12, 0.022], size: 0.024 }, // 安装位
  gripped: false,
  gripOpen: 1,          // 1=张开 0=闭合
  stage: 'idle',
  ikSols: [], ikPick: 0,
  showWorkspace: false,
  showFrames: false,    // 坐标链坐标系可视化
  closedLoop: false,    // 闭环模式：用合成相机估计而非真值
  noiseSigma: 0.002,    // 相机位置噪声 sigma (m)
  estTarget: null,      // 估计的目标位置（闭环）
  estError: 0,          // 估计误差 (m)
};
// 简易高斯随机（Box-Muller），用于相机噪声
function gauss() {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
const STAGE_NAMES = {
  idle: '待开始', observe: '识别/定位', pregrasp: '移动到预抓取位', approach: '接近拾取位',
  pick: '拾取确认', lift: '抬起', transfer: '搬运', insert: '对准安装位',
  assemble: '装配确认', completed: '仿真完成', failed: '失败',
};

// ---- 相机/投影 ----
let az = -0.7, elevation = 0.55, zoom = 1, drag = null;
const FOCUS = [0.13, 0, 0.08];
function project(p) {
  const x = p[0] - FOCUS[0], y = p[1] - FOCUS[1], z = p[2] - FOCUS[2];
  const rx = Math.cos(az) * x - Math.sin(az) * y;
  const ry = Math.sin(az) * x + Math.cos(az) * y;
  return [rx, Math.sin(elevation) * ry - Math.cos(elevation) * z];
}

function scenePoints() {
  const st = K.forward(state.angles.psi, state.angles.alpha, state.angles.beta);
  const pts = [[0, 0, 0], st.elbow, st.end_pos, state.object.pos, state.install.pos,
  [0.32, 0.22, 0], [-0.05, -0.22, 0], [0, 0, 0.30]];
  return { st, pts };
}

// ---- 合成相机 + 手眼坐标链（闭环模式用，对齐 arm_sitl/camera.py）----
function currentObjPos() {
  const st = K.forward(state.angles.psi, state.angles.alpha, state.angles.beta);
  return state.gripped ? [st.end_pos[0], st.end_pos[1], st.end_pos[2] - GRASP_DROP] : state.object.pos;
}
function T_base_camera() {
  const q = [state.angles.psi, state.angles.alpha, state.angles.beta];
  return K.mulSe3(K.T_base_tool(q), T_tool_camera);
}
// 仿真一帧相机观测并反算基座系目标估计：T_base_est = T_base_camera · (inv(T_base_camera)·T_true + 噪声)
function cameraEstimate() {
  const Tbc = T_base_camera();
  const Ttrue = K.se3(K.rotRpy([0, 0, state.object.yaw]), currentObjPos());
  const Tcam = K.mulSe3(K.invSe3(Tbc), Ttrue);
  const R = [Tcam[0].slice(0, 3), Tcam[1].slice(0, 3), Tcam[2].slice(0, 3)];
  const t = [Tcam[0][3] + gauss() * state.noiseSigma, Tcam[1][3] + gauss() * state.noiseSigma, Tcam[2][3] + gauss() * state.noiseSigma];
  const Test = K.mulSe3(Tbc, K.se3(R, t));
  return K.trans(Test);
}
function computeEstimate() {
  if (!state.closedLoop) { state.estTarget = null; state.estError = 0; return; }
  state.estTarget = cameraEstimate();
  const truth = currentObjPos();
  state.estError = Math.hypot(state.estTarget[0] - truth[0], state.estTarget[1] - truth[1], state.estTarget[2] - truth[2]);
}
// 演示用的目标参考：闭环用估计，开环用真值
function getTargetRef() {
  return (state.closedLoop && state.estTarget) ? state.estTarget : state.object.pos;
}

function drawScene() {
  const svg = $('scene');
  const w = Math.max(300, svg.parentElement.clientWidth - 20);
  const h = Math.min(560, Math.max(360, w * 0.60));
  svg.setAttribute('viewBox', `0 0 ${w} ${h}`); svg.style.height = h + 'px';
  const { st, pts } = scenePoints();
  const ps = pts.map(project);
  const xs = ps.map(p => p[0]), ys = ps.map(p => p[1]);
  const minx = Math.min(...xs), maxx = Math.max(...xs), miny = Math.min(...ys), maxy = Math.max(...ys);
  const s = zoom * Math.min((w - 90) / Math.max(0.35, maxx - minx), (h - 80) / Math.max(0.35, maxy - miny));
  const point = p => { const r = project(p); return [w / 2 + (r[0] - (maxx + minx) / 2) * s, h / 2 + (r[1] - (miny + maxy) / 2) * s]; };
  const line = (a, b, cls) => { const p = point(a), q = point(b); return `<line class="${cls}" x1="${p[0]}" y1="${p[1]}" x2="${q[0]}" y2="${q[1]}"/>`; };
  const label = (v, text, dx = 8, dy = -9) => { const p = point(v); return `<text x="${Math.max(6, Math.min(w - 90, p[0] + dx))}" y="${Math.max(16, Math.min(h - 8, p[1] + dy))}">${text}</text>`; };
  let html = '<title>实物三自由度机械臂仿真（合成级，非实机）</title>';
  // 网格
  for (let x = -0.05; x <= 0.33; x += 0.06) html += line([x, -0.22, 0], [x, 0.22, 0], 'grid');
  for (let y = -0.22; y <= 0.22; y += 0.06) html += line([-0.05, y, 0], [0.33, y, 0], 'grid');
  // 坐标轴
  const O = [0, 0, 0];
 [['x', [0.10, 0, 0]], ['y', [0, 0.10, 0]], ['z', [0, 0, 0.10]]].forEach(([n, p]) => { html += line(O, p, 'axis') + label(p, n); });
  // workspace 圈
  if (state.showWorkspace) {
    const rmax = L1 + L2; let ring = '';
    for (let i = 0; i <= 60; i++) { const a = i / 60 * 2 * Math.PI; const p = point([rmax * Math.cos(a), rmax * Math.sin(a), 0]); ring += (i ? 'L' : 'M') + p[0] + ' ' + p[1]; }
    html += `<path class="ws" d="${ring}Z"/>`;
  }
  // 臂：基座->肘->末端
  html += line(O, st.elbow, 'arm') + line(st.elbow, st.end_pos, 'arm');
  [O, st.elbow].forEach(p => { const v = point(p); html += `<circle class="joint" cx="${v[0]}" cy="${v[1]}" r="6"/>`; });
  // 末端水平夹爪（两指，沿末端x向两侧，体现保持水平）
  const ex = [st.end_rot[0][0], st.end_rot[1][0], st.end_rot[2][0]];
  const ey = [st.end_rot[0][1], st.end_rot[1][1], st.end_rot[2][1]];
  const P = st.end_pos;
  const gap = 0.012 * state.gripOpen + 0.004;
  const down = [0, 0, -0.022];
  for (const sgn of [-1, 1]) {
    const base = [P[0] + sgn * gap * ey[0], P[1] + sgn * gap * ey[1], P[2] + sgn * gap * ey[2]];
    const tip = [base[0] + down[0], base[1] + down[1], base[2] + down[2]];
    html += line(base, tip, 'gripper');
  }
  const pe = point(P); html += `<circle class="joint end" cx="${pe[0]}" cy="${pe[1]}" r="6"/>` + label(P, '末端 E', -58, -14);
  // 物体（头部样件）与安装位
  const objPos = state.gripped ? [P[0], P[1], P[2] - GRASP_DROP] : state.object.pos;
  cube(objPos, state.object.size, state.gripped ? 'head held' : 'head', point);
  html += label(objPos, state.gripped ? '物体 H(已夹)' : '物体 H', 10, -14);
  cube(state.install.pos, state.install.size, 'socket', point);
  html += label(state.install.pos, '安装位', 8, 28);
  // IK 目标标记
  if (state.ikTarget) { const p = point(state.ikTarget); html += `<circle class="goal" cx="${p[0]}" cy="${p[1]}" r="11"/>` + label(state.ikTarget, 'IK目标', 10, 26); }
  // 闭环估计目标标记
  if (state.closedLoop && state.estTarget) {
    const p = point(state.estTarget);
    html += `<circle class="est" cx="${p[0]}" cy="${p[1]}" r="9"/>` + label(state.estTarget, '相机估计', 10, -22);
  }
  // 坐标链坐标系（base→tool→camera→target），服务 M3 学习
  if (state.showFrames) {
    const q = [state.angles.psi, state.angles.alpha, state.angles.beta];
    const frames = [
      [K.se3([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0]), '基座B', 0.05],
      [K.T_base_tool(q), '工具T', 0.04],
      [T_base_camera(), '相机C', 0.035],
      [K.se3(K.rotRpy([0, 0, state.object.yaw]), currentObjPos()), '目标H', 0.03],
    ];
    for (const [T, name, sc] of frames) {
      const o = K.trans(T);
      const cols = [[T[0][0], T[1][0], T[2][0]], [T[0][1], T[1][1], T[2][1]], [T[0][2], T[1][2], T[2][2]]];
      ['fx', 'fy', 'fz'].forEach((cls, i) => {
        const c = cols[i];
        html += line(o, [o[0] + sc * c[0], o[1] + sc * c[1], o[2] + sc * c[2]], cls);
      });
      html += label(o, name, 6, 16);
    }
  }
  svg.innerHTML = html;

  function cube(pos, size, cls, pt) {
    const verts = [];
    for (const z of [-size, size]) for (const y of [-size, size]) for (const x of [-size, size]) verts.push([pos[0] + x, pos[1] + y, pos[2] + z]);
    for (const f of [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]) {
      const poly = f.map(i => pt(verts[i]).join(',')).join(' ');
      html += `<polygon class="${cls}" points="${poly}"/>`;
    }
  }
}

// ---- 状态面板 ----
function updatePanel() {
  const st = K.forward(state.angles.psi, state.angles.alpha, state.angles.beta);
  $('stage').textContent = STAGE_NAMES[state.stage] || state.stage;
  $('angles').textContent = `ψ=${deg(state.angles.psi).toFixed(1)}°  α=${deg(state.angles.alpha).toFixed(1)}°  β=${deg(state.angles.beta).toFixed(1)}°`;
  $('endpos').textContent = fmt(st.end_pos) + ' m';
  $('reach').textContent = `可达半径 [${K.reachRadius()[0].toFixed(3)}, ${K.reachRadius()[1].toFixed(3)}] m`;
  $('sl_psi').value = deg(state.angles.psi);
  $('sl_alpha').value = deg(state.angles.alpha);
  $('sl_beta').value = deg(state.angles.beta);
  $('v_psi').textContent = deg(state.angles.psi).toFixed(0) + '°';
  $('v_alpha').textContent = deg(state.angles.alpha).toFixed(0) + '°';
  $('v_beta').textContent = deg(state.angles.beta).toFixed(0) + '°';
  if ($('est_pos')) {
    if (state.closedLoop && state.estTarget) {
      $('est_pos').textContent = fmt(state.estTarget) + ' m';
      $('est_err').textContent = (state.estError * 1000).toFixed(1) + ' mm';
    } else { $('est_pos').textContent = '—（开环）'; $('est_err').textContent = '—'; }
  }
}
function render() { computeEstimate(); drawScene(); updatePanel(); }

// ---- 动画 ----
let anim = null;
function animateTo(target, dur, onDone) {
  if (anim) cancelAnimationFrame(anim);
  const from = { ...state.angles }; const t0 = performance.now();
  const ease = t => t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
  function step(now) {
    const t = Math.min(1, (now - t0) / dur); const e = ease(t);
    state.angles.psi = from.psi + (target.psi - from.psi) * e;
    state.angles.alpha = from.alpha + (target.alpha - from.alpha) * e;
    state.angles.beta = from.beta + (target.beta - from.beta) * e;
    render();
    if (t < 1) anim = requestAnimationFrame(step);
    else { anim = null; if (onDone) onDone(); }
  }
  anim = requestAnimationFrame(step);
}
// 选与当前最接近的解，避免肘部翻转
function pickSol(sols) {
  if (!sols.length) return null;
  let best = sols[0], bd = 1e9;
  for (const s of sols) {
    const d = Math.abs(s.alpha - state.angles.alpha) + Math.abs(s.beta - state.angles.beta);
    if (d < bd) { bd = d; best = s; }
  }
  return best;
}
function moveEndTo(pos, dur, onDone) {
  const sol = pickSol(K.inverse(pos));
  if (!sol) { state.stage = 'failed'; render(); if (onDone) onDone(false); return; }
  animateTo(sol, dur, () => onDone && onDone(true));
}

// ---- 自动抓取演示（任务状态机）----
let demoRunning = false;
function runDemo() {
  if (demoRunning) return;
  demoRunning = true; state.gripped = false; state.gripOpen = 1;
  state.object.pos = [0.18, 0.10, 0.022];
  const inst = state.install.pos;
  const above = (p, h) => [p[0], p[1], p[2] + h];
  const graspEnd = p => above(p, GRASP_DROP); // 末端到此高度时，夹住的物体正好落在 p
  let liftRef = null;
  const steps = [
    ['observe', 600, null],
    ['pregrasp', 1000, () => moveEndTo(above(getTargetRef(), 0.09), 1000, next)],
    ['approach', 900, () => moveEndTo(graspEnd(getTargetRef()), 900, next)],
    ['pick', 500, () => { state.gripOpen = 0; state.gripped = true; liftRef = currentObjPos(); render(); setTimeout(next, 500); }],
    ['lift', 900, () => moveEndTo(above(liftRef, 0.12), 900, next)],
    ['transfer', 1200, () => moveEndTo(above(inst, 0.10), 1200, next)],
    ['insert', 900, () => moveEndTo(graspEnd(inst), 900, next)],
    ['assemble', 500, () => { state.gripOpen = 1; state.gripped = false; state.object.pos = [...inst]; render(); setTimeout(next, 500); }],
    ['completed', 800, () => moveEndTo([0.16, 0, 0.16], 800, () => { demoRunning = false; })],
  ];
  let i = 0;
  function next() {
    if (i >= steps.length) { demoRunning = false; return; }
    const [name, , fn] = steps[i++];
    state.stage = name; render();
    if (fn) fn(); else setTimeout(next, 400);
  }
  next();
}

// ---- IK 面板 ----
function solveIK() {
  const p = [parseFloat($('ik_x').value), parseFloat($('ik_y').value), parseFloat($('ik_z').value)];
  state.ikTarget = p;
  const sols = K.inverse(p);
  state.ikSols = sols;
  const box = $('ik_result');
  if (!sols.length) { box.innerHTML = `<span class="bad">不可达或无解（超出 [|L1-L2|, L1+L2]）</span>`; render(); return; }
  box.innerHTML = sols.map((s, i) =>
    `<button class="solbtn" data-i="${i}">解${i + 1}: ψ=${deg(s.psi).toFixed(1)}° α=${deg(s.alpha).toFixed(1)}° β=${deg(s.beta).toFixed(1)}°</button>`).join('');
  box.querySelectorAll('.solbtn').forEach(b => b.onclick = () => {
    const s = sols[+b.dataset.i]; animateTo(s, 800, null);
  });
  render();
}

// ---- 交互绑定 ----
function bind() {
  const svg = $('scene');
  svg.addEventListener('mousedown', e => { drag = { x: e.clientX, y: e.clientY, az, elevation }; });
  window.addEventListener('mousemove', e => {
    if (!drag) return;
    az = drag.az + (e.clientX - drag.x) * 0.008;
    elevation = Math.max(0.1, Math.min(1.4, drag.elevation + (e.clientY - drag.y) * 0.006));
    drawScene();
  });
  window.addEventListener('mouseup', () => drag = null);
  svg.addEventListener('wheel', e => { e.preventDefault(); zoom = Math.max(0.4, Math.min(3, zoom * (e.deltaY < 0 ? 1.1 : 0.9))); drawScene(); }, { passive: false });

  const onSlider = () => {
    if (demoRunning) return;
    state.angles.psi = parseFloat($('sl_psi').value) * DEG;
    state.angles.alpha = parseFloat($('sl_alpha').value) * DEG;
    state.angles.beta = parseFloat($('sl_beta').value) * DEG;
    state.stage = 'idle'; render();
  };
  ['sl_psi', 'sl_alpha', 'sl_beta'].forEach(id => $(id).addEventListener('input', onSlider));

  $('btn_ik').onclick = solveIK;
  $('btn_demo').onclick = runDemo;
  $('btn_reset').onclick = () => {
    if (anim) cancelAnimationFrame(anim);
    demoRunning = false; state.gripped = false; state.gripOpen = 1; state.stage = 'idle';
    state.angles = { psi: 0, alpha: 0.5, beta: 0.5 };
    state.object.pos = [0.18, 0.10, 0.022]; state.ikTarget = null; $('ik_result').innerHTML = '';
    render();
  };
  $('btn_ws').onclick = () => { state.showWorkspace = !state.showWorkspace; drawScene(); };
  $('btn_frames').onclick = () => { state.showFrames = !state.showFrames; $('btn_frames').classList.toggle('on', state.showFrames); drawScene(); };
  $('btn_cl').onclick = () => { state.closedLoop = !state.closedLoop; $('btn_cl').classList.toggle('on', state.closedLoop); render(); };
  $('sl_noise').addEventListener('input', () => {
    state.noiseSigma = parseFloat($('sl_noise').value) / 1000;
    $('v_noise').textContent = parseFloat($('sl_noise').value).toFixed(1) + ' mm';
    render();
  });
  window.addEventListener('resize', drawScene);
}

// ---- 启动 ----
window.addEventListener('DOMContentLoaded', () => {
  const t = K.selfTest();
  $('selftest').innerHTML = t.ok
    ? `<span class="ok">数学自检 PASS</span> · ${t.why}`
    : `<span class="bad">数学自检 FAIL: ${t.why}</span>`;
  bind(); render();
});
