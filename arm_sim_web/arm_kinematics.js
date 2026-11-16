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

  // ---------- SE(3) 助手（坐标链 / 相机仿真用）----------
  function rotRpy(rpy) {
    const [r, p, y] = rpy;
    const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
    const Rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]];
    const Ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]];
    const Rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]];
    return mul3(Rz, mul3(Ry, Rx));
  }
  function mul3(A, B) {
    const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
      C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
    return C;
  }
  function se3(R, t) { return [[R[0][0], R[0][1], R[0][2], t[0]], [R[1][0], R[1][1], R[1][2], t[1]], [R[2][0], R[2][1], R[2][2], t[2]], [0, 0, 0, 1]]; }
  function mulSe3(A, B) {
    const C = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
    for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++)
      C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j] + A[i][3] * B[3][j];
    return C;
  }
  function invSe3(T) {
    const R = [[T[0][0], T[0][1], T[0][2]], [T[1][0], T[1][1], T[1][2]], [T[2][0], T[2][1], T[2][2]]];
    const t = [T[0][3], T[1][3], T[2][3]];
    const Rt = [[R[0][0], R[1][0], R[2][0]], [R[0][1], R[1][1], R[2][1]], [R[0][2], R[1][2], R[2][2]]];
    const nt = [-(Rt[0][0] * t[0] + Rt[0][1] * t[1] + Rt[0][2] * t[2]), -(Rt[1][0] * t[0] + Rt[1][1] * t[1] + Rt[1][2] * t[2]), -(Rt[2][0] * t[0] + Rt[2][1] * t[1] + Rt[2][2] * t[2])];
    return se3(Rt, nt);
  }
  function T_base_tool(q, l1 = L1, l2 = L2) {
    const st = forward(q[0], q[1], q[2], l1, l2);
    return se3(st.end_rot, st.end_pos);
  }
  function trans(T) { return [T[0][3], T[1][3], T[2][3]]; }

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
    // SE(3) 往返 + 坐标链零噪声一致性
    const T = T_base_tool([0.3, 0.4, 0.2]);
    const I = mulSe3(T, invSe3(T));
    for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++)
      if (Math.abs(I[i][j] - (i === j ? 1 : 0)) > 1e-9) return { ok: false, why: 'SE3 往返' };
    return { ok: true, why: 'FK对齐C++3例 + 末端水平 + IK多解往返 + 不可达 + SE3往返' };
  }

  return { L1, L2, forward, inverse, reachable, reachRadius, selfTest,
           rotRpy, se3, mulSe3, invSe3, T_base_tool, trans, mul3 };
});
