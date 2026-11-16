"""diesel 三自由度机械臂仿真台 —— 可视化 + 手算验证（合成/仿真级，非实机）。

用法（在 tools/arm_sim_lab 下）：
  python3 arm_sim.py --verify                 # 核对 3 道手算练习 + C++ 用例
  python3 arm_sim.py --fk 0 90 0              # 正解：ψ α β（度），画臂
  python3 arm_sim.py --ik 0.149 0 0.149       # 逆解：目标 xyz（米），画两解
  python3 arm_sim.py --workspace              # 画可达 workspace
  python3 arm_sim.py --interactive            # 滑条拖动 ψ/α/β 实时看臂
  加 --save out.png 只存图不弹窗；加 --no-show 只算不画。

数学全部来自 arm_kinematics.py（对齐你的 C++）。本文件只负责画与交互。
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

import arm_kinematics as ak

DEG = np.pi / 180.0


def _setup_mpl(show: bool):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (注册 3d)
    # 自动选一个系统中文字体，避免标签变方框
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in ("Noto Sans CJK SC", "Noto Sans CJK JP", "Droid Sans Fallback",
                 "AR PL UMing CN", "WenQuanYi Micro Hei"):
        if name in available:
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False
    return plt


def _equal_axis(ax, pts, margin=0.05):
    pts = np.asarray(pts)
    lo = pts.min(axis=0) - margin
    hi = pts.max(axis=0) + margin
    center = (lo + hi) / 2
    span = max(hi - lo) / 2
    ax.set_xlim(center[0] - span, center[0] + span)
    ax.set_ylim(center[1] - span, center[1] + span)
    ax.set_zlim(center[2] - span, center[2] + span)
    ax.set_xlabel("x (m) 前")
    ax.set_ylabel("y (m) 左")
    ax.set_zlabel("z (m) 上")


def draw_arm(ax, a: ak.Angles, g: ak.Geometry, color="tab:blue", label=None, alpha=1.0):
    st = ak.forward(a, g)
    O = np.zeros(3)
    E = st.elbow
    P = st.end_pos
    ax.plot(*zip(O, E), color=color, lw=4, alpha=alpha,
            label=label if label else None)
    ax.plot(*zip(E, P), color=color, lw=4, alpha=alpha)
    ax.scatter(*O, color="k", s=40)          # 基座/肩
    ax.scatter(*E, color=color, s=30, alpha=alpha)  # 肘
    # 末端水平指示：沿末端 x 轴画一小段（体现"保持水平、仅随 yaw"）
    ex = st.end_rot[:, 0]
    ax.plot(*zip(P, P + 0.03 * ex), color=color, lw=2, alpha=alpha)
    ax.scatter(*P, color=color, marker="o", s=50, alpha=alpha)
    return st


def _fig3d(plt, title):
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title(title)
    return fig, ax


def cmd_fk(args, plt):
    g = ak.Geometry()
    a = ak.Angles(psi=args.fk[0] * DEG, alpha=args.fk[1] * DEG, beta=args.fk[2] * DEG)
    st = ak.forward(a, g)
    print(f"ψ={args.fk[0]:.1f}° α={args.fk[1]:.1f}° β={args.fk[2]:.1f}°")
    print(f"肘部   = {np.round(st.elbow, 4)} m")
    print(f"末端轴点 = {np.round(st.end_pos, 4)} m  (末端保持水平)")
    if args.no_show:
        return
    fig, ax = _fig3d(plt, "正运动学 FK")
    draw_arm(ax, a, g, label="arm")
    _equal_axis(ax, [np.zeros(3), st.elbow, st.end_pos])
    ax.legend()
    _finish(fig, args, plt)


def cmd_ik(args, plt):
    g = ak.Geometry()
    target = np.array(args.ik, dtype=float)
    sols = ak.inverse(target, g)
    print(f"目标 = {target} m")
    if not ak.reachable(target, g):
        print("不可达：超出 [|L1-L2|, L1+L2] 球壳。")
    if not sols:
        print("无解（不可达或被限位过滤）。")
    for i, a in enumerate(sols):
        print(f"解{i+1}: ψ={a.psi/DEG:.2f}° α={a.alpha/DEG:.2f}° β={a.beta/DEG:.2f}°")
    if args.no_show:
        return
    fig, ax = _fig3d(plt, "逆运动学 IK（肘上/肘下多解）")
    colors = ["tab:blue", "tab:orange"]
    pts = [np.zeros(3), target]
    for i, a in enumerate(sols):
        st = draw_arm(ax, a, g, color=colors[i % 2], label=f"解{i+1}", alpha=0.85)
        pts += [st.elbow, st.end_pos]
    ax.scatter(*target, color="red", marker="*", s=160, label="目标")
    _equal_axis(ax, pts)
    ax.legend()
    _finish(fig, args, plt)


def cmd_workspace(args, plt):
    g = ak.Geometry()
    rmin, rmax = ak.reach_radius(g)
    print(f"可达半径范围 [{rmin:.3f}, {rmax:.3f}] m（平面两连杆）")
    if args.no_show:
        return
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111)
    th = np.linspace(0, 2 * np.pi, 400)
    ax.plot(rmax * np.cos(th), rmax * np.sin(th), "b-", label=f"外边界 r={rmax:.3f}")
    if rmin > 1e-6:
        ax.plot(rmin * np.cos(th), rmin * np.sin(th), "b--", label=f"内边界 r={rmin:.3f}")
    ax.set_aspect("equal")
    ax.set_xlabel("r (m)")
    ax.set_ylabel("z (m)")
    ax.set_title("可达 workspace（r-z 平面，绕 z 轴旋转成球壳）")
    ax.grid(True)
    ax.legend()
    _finish(fig, args, plt)


def cmd_verify(args, plt):
    g = ak.Geometry()
    print("=" * 60)
    print("手算验证（合成级）—— 先自己算，再看这里核对")
    print("=" * 60)

    print("\n[练习1] 目标 (0.298, 0, 0)  手臂完全伸直")
    t1 = np.array([0.298, 0, 0])
    r, z = 0.298, 0.0
    c2 = (r * r + z * z - g.l1 ** 2 - g.l2 ** 2) / (2 * g.l1 * g.l2)
    print(f"  c2 = {c2:.6f}  (应为 +1，边界)")
    for a in ak.inverse(t1, g):
        print(f"  解: ψ={a.psi/DEG:.2f}° α={a.alpha/DEG:.2f}° β={a.beta/DEG:.2f}°  (应 α=β=0)")
    print("  注：c2=1 时 ±acos 两解重合（奇异位形），所以打印两条相同的解。")

    print("\n[练习2] 目标 (0.5, 0, 0)")
    t2 = np.array([0.5, 0, 0])
    print(f"  reachable = {ak.reachable(t2, g)}  (应 False，0.5 > 0.298)")
    print(f"  解个数 = {len(ak.inverse(t2, g))}  (应 0)")

    print("\n[多解示例] 目标 (0.149, 0, 0.149)")
    t3 = np.array([0.149, 0, 0.149])
    sols = ak.inverse(t3, g)
    for a in sols:
        print(f"  解: ψ={a.psi/DEG:.2f}° α={a.alpha/DEG:.2f}° β={a.beta/DEG:.2f}°")
    print("  应为 (α=90°,β=0°) 与 (α=0°,β=90°) 两组 —— 同一点，不同关节角")

    print("\n[练习3] 为什么实物不能给末端任意 pitch？")
    print("  实物只有 3 主动自由度(ψ,α,β)+连杆保水平，无腕 pitch。")
    print("  末端姿态被约束为水平且仅随 yaw，不能独立俯仰 → 不能任意满足六维姿态。")

    print("\n[C++ 对齐] FK 三例 + 末端水平 + IK 往返")
    ak._self_test()
    print("=" * 60)


def cmd_interactive(args, plt):
    from matplotlib.widgets import Slider
    g = ak.Geometry()
    fig, ax = _fig3d(plt, "拖动滑条看正运动学（ψ/α/β，度）")
    fig.subplots_adjust(bottom=0.28)
    state = {"a": ak.Angles(0, 0.3, 0.3)}
    drawn = []

    def redraw():
        for art in drawn:
            art.remove()
        drawn.clear()
        st = ak.forward(state["a"], g)
        O, E, P = np.zeros(3), st.elbow, st.end_pos
        drawn.append(ax.plot(*zip(O, E), color="tab:blue", lw=4)[0])
        drawn.append(ax.plot(*zip(E, P), color="tab:blue", lw=4)[0])
        drawn.append(ax.scatter(*E, color="tab:blue", s=30))
        drawn.append(ax.scatter(*P, color="tab:blue", s=50))
        ex = st.end_rot[:, 0]
        drawn.append(ax.plot(*zip(P, P + 0.03 * ex), color="tab:blue", lw=2)[0])
        _equal_axis(ax, [O, E, P])
        ax.set_title(f"末端 = {np.round(P,3)} m")
        fig.canvas.draw_idle()

    ax_psi = fig.add_axes([0.15, 0.16, 0.7, 0.03])
    ax_alpha = fig.add_axes([0.15, 0.10, 0.7, 0.03])
    ax_beta = fig.add_axes([0.15, 0.04, 0.7, 0.03])
    s_psi = Slider(ax_psi, "ψ yaw", -180, 180, valinit=0)
    s_alpha = Slider(ax_alpha, "α 大臂", -90, 180, valinit=0.3 / DEG)
    s_beta = Slider(ax_beta, "β 小臂", -180, 180, valinit=0.3 / DEG)

    def update(_):
        state["a"] = ak.Angles(s_psi.val * DEG, s_alpha.val * DEG, s_beta.val * DEG)
        redraw()

    s_psi.on_changed(update)
    s_alpha.on_changed(update)
    s_beta.on_changed(update)
    redraw()
    _finish(fig, args, plt)


def _finish(fig, args, plt):
    if args.save:
        fig.savefig(args.save, dpi=120, bbox_inches="tight")
        print(f"已存图: {args.save}")
    if not args.no_show and not args.save:
        plt.show()
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description="diesel 三自由度机械臂仿真台（合成/仿真级）")
    p.add_argument("--fk", nargs=3, type=float, metavar=("PSI", "ALPHA", "BETA"),
                   help="正解：三个几何角（度）")
    p.add_argument("--ik", nargs=3, type=float, metavar=("X", "Y", "Z"),
                   help="逆解：目标位置（米）")
    p.add_argument("--workspace", action="store_true", help="画可达 workspace")
    p.add_argument("--verify", action="store_true", help="核对 3 道手算练习 + C++ 用例")
    p.add_argument("--interactive", action="store_true", help="滑条交互")
    p.add_argument("--save", type=str, help="存图路径（不弹窗）")
    p.add_argument("--no-show", action="store_true", help="只计算打印，不画图")
    args = p.parse_args()

    if args.verify:
        cmd_verify(args, None)
        return
    if not any([args.fk, args.ik, args.workspace, args.interactive]):
        p.print_help()
        return
    show = not (args.save or args.no_show)
    plt = _setup_mpl(show)
    if args.fk:
        cmd_fk(args, plt)
    elif args.ik:
        cmd_ik(args, plt)
    elif args.workspace:
        cmd_workspace(args, plt)
    elif args.interactive:
        cmd_interactive(args, plt)


if __name__ == "__main__":
    main()
