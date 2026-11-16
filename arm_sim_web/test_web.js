// 网页模拟器无头功能测试：node test_web.js
// 轻量 DOM 桩 + 虚拟时钟，验证渲染/坐标系/闭环/噪声/演示。合成/仿真级。
'use strict';
const path = require('path');
const dir = __dirname;
let now = 0, rafQ = [], timerQ = [], rafId = 1, timerId = 1;
global.performance = { now: () => now };
global.requestAnimationFrame = cb => { rafQ.push({ id: rafId, cb }); return rafId++; };
global.cancelAnimationFrame = id => { rafQ = rafQ.filter(x => x.id !== id); };
global.setTimeout = (fn, d) => { timerQ.push({ id: timerId, fn, at: now + (d || 0) }); return timerId++; };
function flush(maxIter = 6000) {
  for (let i = 0; i < maxIter; i++) {
    if (!rafQ.length && !timerQ.length) return;
    now += 20;
    const rafs = rafQ; rafQ = []; rafs.forEach(x => x.cb(now));
    const due = timerQ.filter(t => t.at <= now); timerQ = timerQ.filter(t => t.at > now);
    due.forEach(t => t.fn());
  }
  throw new Error('flush 超限');
}
const els = {};
function mkEl(id) {
  return { id, value: '0', textContent: '', innerHTML: '', style: {}, dataset: {}, onclick: null,
    parentElement: { clientWidth: 640 }, classList: { toggle() {}, add() {}, remove() {} },
    setAttribute() {}, addEventListener() {}, querySelectorAll() { return []; } };
}
const get = id => (els[id] || (els[id] = mkEl(id)));
const domH = {}, winH = {};
global.document = { getElementById: get, addEventListener: (e, f) => { (domH[e] = domH[e] || []).push(f); } };
global.window = { addEventListener: (e, f) => { (winH[e] = winH[e] || []).push(f); } };
global.window.ArmKin = require(path.join(dir, 'arm_kinematics.js'));
require(path.join(dir, 'app.js'));
(winH['DOMContentLoaded'] || []).forEach(f => f());
const assert = (c, m) => { if (!c) { console.error('FAIL:', m); process.exit(1); } console.log('PASS:', m); };

assert(get('selftest').innerHTML.includes('PASS'), '数学自检 PASS（含 SE3）');
assert(get('scene').innerHTML.includes('class="arm"'), '渲染臂');
get('btn_frames').onclick();
assert(get('scene').innerHTML.includes('class="fx"'), '坐标系开关画出 x 轴');
assert(get('scene').innerHTML.includes('相机C'), '坐标系含相机C标签');
get('btn_cl').onclick();
assert(get('est_pos').textContent.includes('m'), '闭环模式估计目标有值');
assert(get('scene').innerHTML.includes('class="est"'), '画出相机估计标记');
get('btn_cl').onclick(); // 关闭环
get('btn_demo').onclick(); flush();
assert(get('stage').textContent === '仿真完成', '开环演示跑到完成');
get('btn_reset').onclick();
get('btn_cl').onclick(); // 开闭环
get('btn_demo').onclick(); flush();
assert(get('stage').textContent === '仿真完成', '闭环演示跑到完成');
console.log('\n网页模拟器功能全部通过 ✓（合成/仿真级）');
