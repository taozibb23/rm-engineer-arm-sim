"""diesel 三自由度机械臂运动学 + SE(3) 助手（合成/仿真级）。
FK/IK 与 tools/arm_sim_lab/arm_kinematics.py 同源，对齐 C++ level_arm.cpp。
零 ROS。角度内部 rad。
"""
from __future__ import annotations
import numpy as np

L1, L2 = 0.149, 0.149
_EPS_R, _ACOS_TOL = 1e-8, 1e-9


# ---------- SE(3) ----------
def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_rpy(rpy):
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def se3(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def inv_se3(T):
    R = T[:3, :3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ T[:3, 3]
    return Ti


def pose_from_cfg(d):
    return se3(rot_rpy(d.get("rpy_rad", [0, 0, 0])), np.array(d["translation_m"], float))


# ---------- 运动学 ----------
def forward(psi, alpha, beta, l1=L1, l2=L2):
    ur, uz = l1 * np.cos(alpha), l1 * np.sin(alpha)
    radius = ur + l2 * np.cos(beta)
    height = uz + l2 * np.sin(beta)
    cy, sy = np.cos(psi), np.sin(psi)
    return {
        "elbow": np.array([ur * cy, ur * sy, uz]),
        "end_pos": np.array([radius * cy, radius * sy, height]),
        "end_rot": rot_z(psi),
    }


def T_base_tool(q, l1=L1, l2=L2):
    """正运动学 -> 工具(末端轴点)位姿 4x4。q=[psi,alpha,beta]。"""
    st = forward(q[0], q[1], q[2], l1, l2)
    return se3(st["end_rot"], st["end_pos"])


def reachable(p, l1=L1, l2=L2):
    r = np.hypot(p[0], p[1]); z = p[2]
    c2 = (r * r + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    return -1 - _ACOS_TOL <= c2 <= 1 + _ACOS_TOL


def inverse(p, l1=L1, l2=L2, limits=None, tol=1e-6):
    """位置逆解（末端水平）。返回解列表 [{psi,alpha,beta}]，无解 []。"""
    p = np.asarray(p, float)
    r = np.hypot(p[0], p[1]); z = p[2]
    c2 = (r * r + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    if c2 < -1 - _ACOS_TOL or c2 > 1 + _ACOS_TOL:
        return []
    psi = float(np.arctan2(p[1], p[0])) if r > _EPS_R else 0.0
    sols = []
    for sign in (-1, 1):
        th2 = sign * float(np.arccos(np.clip(c2, -1, 1)))
        alpha = float(np.arctan2(z, r) - np.arctan2(l2 * np.sin(th2), l1 + l2 * np.cos(th2)))
        beta = alpha + th2
        q = {"psi": psi, "alpha": alpha, "beta": beta}
        if limits and not _in_limits(q, limits):
            continue
        chk = forward(psi, alpha, beta, l1, l2)["end_pos"]
        if np.linalg.norm(chk - p) < tol:
            sols.append(q)
    return sols


def _in_limits(q, limits):
    for k in ("psi", "alpha", "beta"):
        lo, hi = limits[k]
        if not (lo <= q[k] <= hi):
            return False
    return True


def pick_closest(sols, q_cur):
    if not sols:
        return None
    return min(sols, key=lambda s: abs(s["alpha"] - q_cur[1]) + abs(s["beta"] - q_cur[2]))


def q_to_vec(q):
    return np.array([q["psi"], q["alpha"], q["beta"]])


def self_test():
    assert np.linalg.norm(forward(0, 0, 0)["end_pos"] - [0.298, 0, 0]) < 1e-12
    assert np.linalg.norm(forward(0, np.pi / 2, 0)["end_pos"] - [0.149, 0, 0.149]) < 1e-12
    assert len(inverse([0.149, 0, 0.149])) == 2
    assert inverse([0.5, 0, 0]) == []
    T = T_base_tool([0.3, 0.4, 0.2])
    assert np.linalg.norm(T @ inv_se3(T) - np.eye(4)) < 1e-9
    return "PASS: FK对齐C++ + IK多解 + 不可达 + SE3往返"


if __name__ == "__main__":
    print(self_test())
