/* diesel 实物三自由度机械臂 —— 纯运动学核心（合成/仿真级，非实机）。
 * 移植自 tools/arm_sim_lab/arm_kinematics.py（已对齐 C++ level_arm.cpp）。
 * 约定：psi=底座yaw, alpha=大臂仰角, beta=小臂仰角(均相对水平, rad)；
 *       L1=L2=0.149m；连杆保末端水平，末端姿态仅随 yaw；输出为小臂末端轴点。
 * 零依赖。浏览器挂到 window.ArmKin，node 下 module.exports。
 */
(function (root, factory) {
  const api = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.ArmKin = api;
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';
  const L1 = 0.149, L2 = 0.149;
  const EPS_R = 1e-8, ACOS_TOL = 1e-9;
  const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

  function forward(psi, alpha, beta, l1 = L1, l2 = L2) {
    const ur = l1 * Math.cos(alpha), uz = l1 * Math.sin(alpha);
    const radius = ur + l2 * Math.cos(beta);
    const height = uz + l2 * Math.sin(beta);
    const cy = Math.cos(psi), sy = Math.sin(psi);
    return {
      elbow: [ur * cy, ur * sy, uz],
      end_pos: [radius * cy, radius * sy, height],
      end_rot: [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]],
    };
  }

  function reachRadius(l1 = L1, l2 = L2) { return [Math.abs(l1 - l2), l1 + l2]; }

  function reachable(p, l1 = L1, l2 = L2) {
    const r = Math.hypot(p[0], p[1]), z = p[2];
    const c2 = (r * r + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    return c2 >= -1 - ACOS_TOL && c2 <= 1 + ACOS_TOL;
  }

  // 逆解：位置 -> 几何角（末端水平）。返回 0/1/2 个解，无解返回 []。
  function inverse(p, l1 = L1, l2 = L2, verifyTol = 1e-6) {
    const r = Math.hypot(p[0], p[1]), z = p[2];
    const c2 = (r * r + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    if (c2 < -1 - ACOS_TOL || c2 > 1 + ACOS_TOL) return [];
    const psi = r > EPS_R ? Math.atan2(p[1], p[0]) : 0;
    const sols = [];
    for (const sign of [-1, 1]) {
      const th2 = sign * Math.acos(clamp(c2, -1, 1));
      const alpha = Math.atan2(z, r) - Math.atan2(l2 * Math.sin(th2), l1 + l2 * Math.cos(th2));
      const beta = alpha + th2;
      const chk = forward(psi, alpha, beta, l1, l2);
      const d = Math.hypot(chk.end_pos[0] - p[0], chk.end_pos[1] - p[1], chk.end_pos[2] - p[2]);
      if (d < verifyTol) sols.push({ psi, alpha, beta });
    }
    return sols;
  }

  // 自检：对齐 C++ test_arm_geometry.cpp
  function selfTest() {
    const near = (a, b, t = 1e-9) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < t;
    const cases = [
      [[0, 0, 0], [0.298, 0, 0]],
      [[Math.PI / 2, 0, 0], [0, 0.298, 0]],
      [[0, Math.PI / 2, 0], [0.149, 0, 0.149]],
    ];
    for (const [a, e] of cases) {
      if (!near(forward(...a).end_pos, e, 1e-12)) return { ok: false, why: 'FK ' + a };
    }
    const rot = forward(0, Math.PI / 2, 0).end_rot;
    if (Math.abs(rot[0][0] - 1) > 1e-12 || Math.abs(rot[1][1] - 1) > 1e-12) return { ok: false, why: '末端水平' };
    const sols = inverse([0.149, 0, 0.149]);
    if (sols.length !== 2) return { ok: false, why: 'IK 多解个数=' + sols.length };
    for (const s of sols) if (!near(forward(s.psi, s.alpha, s.beta).end_pos, [0.149, 0, 0.149])) return { ok: false, why: 'IK 往返' };
    if (inverse([0.5, 0, 0]).length !== 0) return { ok: false, why: '不可达应无解' };
    return { ok: true, why: 'FK对齐C++3例 + 末端水平 + IK多解往返 + 不可达' };
  }

  return { L1, L2, forward, inverse, reachable, reachRadius, selfTest };
});
