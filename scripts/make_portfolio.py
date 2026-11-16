"""生成作品集可视化：抓取演示动图 + 静态图解（合成/仿真级）。
用法：python3 scripts/make_portfolio.py   （输出到 docs/figs/）
可复现证据：本脚本 + arm_sitl/kinematics.py 即可重新生成全部图。
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "arm_sitl"))
import kinematics as kin

FIGDIR = os.path.join(ROOT, "docs", "figs")
os.makedirs(FIGDIR, exist_ok=True)
L1, L2 = kin.L1, kin.L2
GRASP_DROP = 0.030
OBJ = np.array([0.18, 0.10, 0.022])
INST = np.array([0.18, -0.12, 0.022])
HOME = np.array([0.0, 0.5, 0.5])


def ik_at(pos, q_prev):
    sols = kin.inverse(pos, L1, L2)
    if not sols:
        return q_prev
    return kin.q_to_vec(kin.pick_closest(sols, q_prev))


def trajectory():
    """生成抓取序列的关节轨迹 + 物体状态。"""
    above = lambda p, h: np.array([p[0], p[1], p[2] + h])
    waypoints = [
        (" home", HOME, False, 1),
        ("预抓取", ik_at(above(OBJ, 0.09), HOME), False, 1),
        ("接近", ik_at(above(OBJ, GRASP_DROP), HOME), False, 1),
        ("拾取", ik_at(above(OBJ, GRASP_DROP), HOME), True, 0),
        ("抬起", ik_at(above(OBJ, 0.12), HOME), True, 0),
        ("搬运", ik_at(above(INST, 0.10), HOME), True, 0),
        ("放置", ik_at(above(INST, GRASP_DROP), HOME), True, 0),
        ("完成", ik_at(above(INST, GRASP_DROP), HOME), False, 1),
        (" home", HOME, False, 1),
    ]
    frames = []
    per = 12
    for i in range(len(waypoints) - 1):
        name, q1, g1, o1 = waypoints[i]
        _, q2, g2, o2 = waypoints[i + 1]
        for k in range(per):
            t = k / per
            q = q1 + (q2 - q1) * t
            gripped = g1 if t < 0.5 else g2
            grip_open = o1 if t < 0.5 else o2
            frames.append((name, q, gripped, grip_open))
    frames.append(waypoints[-1])
    return frames


def draw_cube(ax, pos, size, color, alpha=0.6):
    x, y, z = pos
    s = size
    v = np.array([[x - s, y - s, z - s], [x + s, y - s, z - s], [x + s, y + s, z - s], [x - s, y + s, z - s],
                  [x - s, y - s, z + s], [x + s, y - s, z + s], [x + s, y + s, z + s], [x - s, y + s, z + s]])
    faces = [[v[0], v[1], v[2], v[3]], [v[4], v[5], v[6], v[7]], [v[0], v[1], v[5], v[4]],
             [v[2], v[3], v[7], v[6]], [v[1], v[2], v[6], v[5]], [v[0], v[3], v[7], v[4]]]
    ax.add_collection3d(Poly3DCollection(faces, facecolors=color, edgecolors="k", linewidths=0.4, alpha=alpha))


def setup_ax(ax, title):
    ax.clear()
    ax.set_xlim(-0.05, 0.32); ax.set_ylim(-0.22, 0.22); ax.set_zlim(0, 0.30)
    ax.set_xlabel("x(m)"); ax.set_ylabel("y(m)"); ax.set_zlabel("z(m)")
    ax.set_title(title, fontsize=11)
    try:
        ax.set_box_aspect((0.37, 0.44, 0.30))
    except Exception:
        pass


def draw_arm(ax, q, gripped, grip_open, obj_pos, title):
    setup_ax(ax, title)
    st = kin.forward(q[0], q[1], q[2], L1, L2)
    O = np.zeros(3); E = st["elbow"]; P = st["end_pos"]
    ax.plot(*zip(O, E), color="#4da3ff", lw=4)
    ax.plot(*zip(E, P), color="#4da3ff", lw=4)
    ax.scatter(*O, color="k", s=30); ax.scatter(*E, color="#4da3ff", s=25)
    # 夹爪
    ey = st["end_rot"][:, 1]
    gap = 0.012 * grip_open + 0.004
    for sgn in (-1, 1):
        b = P + sgn * gap * ey
        tip = b + np.array([0, 0, -0.022])
        ax.plot(*zip(b, tip), color="#38d39f", lw=2)
    ax.scatter(*P, color="#38d39f", s=35)
    draw_cube(ax, obj_pos, 0.020, "#e0a85a", 0.8 if gripped else 0.6)
    draw_cube(ax, INST, 0.024, "#5a9bd5", 0.25)


def make_gif():
    frames = trajectory()
    fig = plt.figure(figsize=(6, 5))
    ax = fig.add_subplot(111, projection="3d")

    def update(i):
        name, q, gripped, grip_open = frames[i]
        if gripped:
            obj = kin.forward(q[0], q[1], q[2])["end_pos"] - np.array([0, 0, GRASP_DROP])
        elif name == "完成":
            obj = INST
        else:
            obj = OBJ
        draw_arm(ax, q, gripped, grip_open, obj, f"抓取演示（合成/仿真） · {name}")
        return []

    anim = FuncAnimation(fig, update, frames=len(frames), interval=80, blit=False)
    out = os.path.join(FIGDIR, "grasp_demo.gif")
    anim.save(out, writer=PillowWriter(fps=12))
    plt.close(fig)
    print("生成动图:", out, f"({len(frames)} 帧)")


def draw_frame_axes(ax, T, name, sc=0.04):
    o = T[:3, 3]
    cols = [T[:3, 0], T[:3, 1], T[:3, 2]]
    for c, col in zip(cols, ["#ff5555", "#44dd66", "#4488ff"]):
        ax.plot(*zip(o, o + sc * c), color=col, lw=2)
    ax.text(*o, name, fontsize=8)


def make_static():
    fig = plt.figure(figsize=(12, 9))
    # 1) 臂 + 坐标链
    ax1 = fig.add_subplot(221, projection="3d")
    q = np.array([0.4, 0.6, 0.3])
    draw_arm(ax1, q, False, 1, OBJ, "① 臂与坐标链 base→tool→camera→target")
    Tbt = kin.T_base_tool(q)
    Ttc = kin.se3(kin.rot_rpy([np.pi, 0, 0]), [0, 0, 0])
    Tbc = Tbt @ Ttc
    Ttarget = kin.se3(kin.rot_z(0), OBJ)
    draw_frame_axes(ax1, np.eye(4), "B", 0.05)
    draw_frame_axes(ax1, Tbt, "T")
    draw_frame_axes(ax1, Tbc, "C", 0.035)
    draw_frame_axes(ax1, Ttarget, "H", 0.03)
    # 2) IK 多解
    ax2 = fig.add_subplot(222, projection="3d")
    setup_ax(ax2, "② IK 多解：同一点，肘上/肘下两组角")
    target = np.array([0.149, 0, 0.149])
    for sol, col in zip(kin.inverse(target, L1, L2), ["#4da3ff", "#ffa500"]):
        qq = kin.q_to_vec(sol)
        st = kin.forward(*qq)
        ax2.plot(*zip(np.zeros(3), st["elbow"], st["end_pos"]), color=col, lw=3)
    ax2.scatter(*target, color="red", marker="*", s=120)
    # 3) workspace
    ax3 = fig.add_subplot(223)
    rmin, rmax = abs(L1 - L2), L1 + L2
    th = np.linspace(0, 2 * np.pi, 300)
    ax3.plot(rmax * np.cos(th), rmax * np.sin(th), "b-", label=f"外边界 r={rmax:.3f}m")
    ax3.fill(rmax * np.cos(th), rmax * np.sin(th), alpha=0.1)
    ax3.set_aspect("equal"); ax3.grid(True); ax3.legend()
    ax3.set_xlabel("r(m)"); ax3.set_ylabel("z(m)")
    ax3.set_title("③ 可达 workspace（绕 z 轴成球壳）")
    # 4) 正运动学曲线
    ax4 = fig.add_subplot(224)
    alphas = np.linspace(-0.5, 2.5, 200)
    for beta, col in [(0.0, "#4da3ff"), (0.5, "#38d39f"), (1.0, "#ffa500")]:
        zs = L1 * np.sin(alphas) + L2 * np.sin(beta)
        rs = L1 * np.cos(alphas) + L2 * np.cos(beta)
        ax4.plot(rs, zs, color=col, label=f"β={beta:.1f}")
    ax4.set_xlabel("r(m)"); ax4.set_ylabel("z(m)"); ax4.grid(True); ax4.legend()
    ax4.set_title("④ 正运动学：α 扫描的末端轨迹")
    fig.suptitle("diesel 三自由度机械臂仿真 · 合成/仿真级 · AI 辅助学习记录", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = os.path.join(FIGDIR, "portfolio.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print("生成图解:", out)


if __name__ == "__main__":
    make_static()
    make_gif()
    print("完成。全部图可由本脚本复现。")
