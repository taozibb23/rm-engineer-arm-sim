"""diesel 实物三自由度机械臂 —— 纯运动学核心（仿真/合成级，非实机）。

对齐来源（逐行核对）：战队 C++ 后端 level_arm.cpp（三自由度正运动学）与实物架构说明。

约定（与 C++ 一致，勿改）：
  psi   ψ  底座 yaw，零位向前 +x、左 +y、上 +z，正 yaw 从上往下看逆时针。
  alpha α  大臂相对水平面的仰角 (rad)。
  beta  β  小臂相对水平面的仰角 (rad)，**不是肘部相对夹角**。
  L1=L2=0.149 m（轴心距，待实测复核）。
  连杆使末端理想保持水平：末端姿态只随 yaw 转，不随大小臂俯仰。
  输出是小臂末端**轴点**，不是抓取中心（夹爪安装偏置未测）。

本文件只做数学，不依赖 ROS / matplotlib，可独立单测。
验证级别：合成/仿真。任何实机精度结论都不能从这里借。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

# ---- 几何参数（来自本人实物说明，轴心距待实测）----
L1_DEFAULT = 0.149  # 大臂 m
L2_DEFAULT = 0.149  # 小臂 m
_EPS_R = 1e-8       # yaw 退化判据（目标落在 z 轴上）
_ACOS_TOL = 1e-9    # 可达边界余量


@dataclass
class Geometry:
    l1: float = L1_DEFAULT
    l2: float = L2_DEFAULT

    def __post_init__(self) -> None:
        if not (np.isfinite(self.l1) and self.l1 > 0 and np.isfinite(self.l2) and self.l2 > 0):
            raise ValueError("link lengths must be finite and positive")


@dataclass
class Angles:
    """几何角 (rad)，不是电机编码器角。"""
    psi: float = 0.0    # 底座 yaw
    alpha: float = 0.0  # 大臂仰角
    beta: float = 0.0   # 小臂仰角

    def as_vec(self) -> np.ndarray:
        return np.array([self.psi, self.alpha, self.beta], dtype=float)


@dataclass
class State:
    elbow: np.ndarray          # 肘部 (3,) m
    end_pos: np.ndarray        # 小臂末端轴点 (3,) m
    end_rot: np.ndarray        # 末端姿态 (3,3)，理想水平 = Rz(psi)


@dataclass
class Limits:
    """关节限位 (rad)。实物限位未测，默认 None 表示不施加。

    一旦实测，填这里；IK 会据此过滤解。未确认前不要把几何角当电机角下发。
    """
    psi: Optional[Tuple[float, float]] = None
    alpha: Optional[Tuple[float, float]] = None
    beta: Optional[Tuple[float, float]] = None

    def allows(self, a: Angles) -> bool:
        for name in ("psi", "alpha", "beta"):
            lim = getattr(self, name)
            if lim is None:
                continue
            v = getattr(a, name)
            if not (lim[0] <= v <= lim[1]):
                return False
        return True


def _rz(psi: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def forward(a: Angles, g: Geometry = Geometry()) -> State:
    """正运动学：几何角 -> 末端位姿。逐行对齐 level_arm.cpp::forward。"""
    v = a.as_vec()
    if not np.all(np.isfinite(v)):
        raise ValueError("geometric angles must be finite")
    upper_r = g.l1 * np.cos(a.alpha)
    upper_z = g.l1 * np.sin(a.alpha)
    radius = upper_r + g.l2 * np.cos(a.beta)
    height = upper_z + g.l2 * np.sin(a.beta)
    cy, sy = np.cos(a.psi), np.sin(a.psi)
    elbow = np.array([upper_r * cy, upper_r * sy, upper_z])
    end_pos = np.array([radius * cy, radius * sy, height])
    end_rot = _rz(a.psi)  # 理想平行连杆：末端水平，仅随 yaw
    return State(elbow=elbow, end_pos=end_pos, end_rot=end_rot)


def reach_radius(g: Geometry = Geometry()) -> Tuple[float, float]:
    """水平+竖直平面内可达半径范围 [|L1-L2|, L1+L2]。"""
    return abs(g.l1 - g.l2), g.l1 + g.l2


def reachable(pos: np.ndarray, g: Geometry = Geometry()) -> bool:
    """目标位置是否在可达球壳内（忽略限位）。"""
    pos = np.asarray(pos, dtype=float)
    if not np.all(np.isfinite(pos)):
        return False
    r = np.hypot(pos[0], pos[1])
    z = pos[2]
    d2 = r * r + z * z
    c2 = (d2 - g.l1 ** 2 - g.l2 ** 2) / (2 * g.l1 * g.l2)
    return -1.0 - _ACOS_TOL <= c2 <= 1.0 + _ACOS_TOL


def inverse(pos: np.ndarray,
            g: Geometry = Geometry(),
            limits: Optional[Limits] = None,
            verify_tol: float = 1e-6) -> List[Angles]:
    """逆运动学：末端位置 -> 几何角（末端保持水平）。

    三自由度只能定位置 + 水平姿态，不能任意满足六维姿态
    （见 REAL_ARM_ARCHITECTURE.md 第 4 步）。

    推导（两连杆，引入肘部相对角 theta2 = beta - alpha）：
        psi = atan2(y, x)
        r   = hypot(x, y)
        c2  = (r^2 + z^2 - L1^2 - L2^2) / (2 L1 L2)   余弦定理
        theta2 = ±acos(c2)                            ± 即肘上/肘下两解
        alpha  = atan2(z, r) - atan2(L2 sinθ2, L1 + L2 cosθ2)
        beta   = alpha + theta2

    返回通过限位且 FK 往返校验的解（可能 0/1/2 个）。无解返回空列表，不伪造零位姿。
    """
    pos = np.asarray(pos, dtype=float)
    if not np.all(np.isfinite(pos)):
        return []
    r = float(np.hypot(pos[0], pos[1]))
    z = float(pos[2])
    d2 = r * r + z * z
    c2 = (d2 - g.l1 ** 2 - g.l2 ** 2) / (2 * g.l1 * g.l2)
    if c2 < -1.0 - _ACOS_TOL or c2 > 1.0 + _ACOS_TOL:
        return []  # 够不到

    # yaw：目标在 z 轴上时退化，取 0 并标注（此处简单取 0）
    psi = float(np.arctan2(pos[1], pos[0])) if r > _EPS_R else 0.0

    sols: List[Angles] = []
    for sign in (-1.0, +1.0):
        theta2 = sign * float(np.arccos(np.clip(c2, -1.0, 1.0)))
        alpha = float(np.arctan2(z, r) - np.arctan2(g.l2 * np.sin(theta2),
                                                   g.l1 + g.l2 * np.cos(theta2)))
        beta = alpha + theta2
        a = Angles(psi=psi, alpha=alpha, beta=beta)
        if limits is not None and not limits.allows(a):
            continue
        # FK 往返校验：位置必须回到目标
        chk = forward(a, g)
        if np.linalg.norm(chk.end_pos - pos) < verify_tol:
            sols.append(a)
    return sols


# ---- 自检：对齐 C++ test_arm_geometry.cpp 的已知答案 ----
def _self_test() -> None:
    g = Geometry()
    pi = np.pi
    cases = [
        (Angles(0, 0, 0), np.array([0.298, 0, 0])),            # 伸直
        (Angles(pi / 2, 0, 0), np.array([0, 0.298, 0])),       # yaw90
        (Angles(0, pi / 2, 0), np.array([0.149, 0, 0.149])),   # 大臂竖直小臂水平
    ]
    for a, expect in cases:
        got = forward(a, g).end_pos
        assert np.linalg.norm(got - expect) < 1e-12, (a, got, expect)
    # 末端水平：大臂竖直时末端姿态仍为单位阵（不随俯仰）
    assert np.linalg.norm(forward(Angles(0, pi / 2, 0), g).end_rot - np.eye(3)) < 1e-12
    # IK 往返 + 多解：目标 (0.149,0,0.149) 应有 (90°,0°) 与 (0°,90°) 两解
    sols = inverse(np.array([0.149, 0, 0.149]), g)
    assert len(sols) == 2, sols
    for a in sols:
        assert np.linalg.norm(forward(a, g).end_pos - np.array([0.149, 0, 0.149])) < 1e-9
    # 不可达
    assert inverse(np.array([0.5, 0, 0]), g) == []
    assert not reachable(np.array([0.5, 0, 0]), g)
    # 非法杆长
    try:
        Geometry(l1=0.0)
        raise AssertionError("should reject zero length")
    except ValueError:
        pass
    print("PASS: FK 对齐 C++ 3 例 + 末端水平 + IK 多解往返 + 不可达 + 非法杆长")


if __name__ == "__main__":
    _self_test()
