"""diesel 机械臂 SITL 仿真平台入口。合成/仿真级，非实机。

用法：
  python3 run.py                      # 纯仿真闭环（默认 config.json）
  python3 run.py --duration 20        # 指定时长
  python3 run.py --config other.json  # 换配置（适配不同硬件）
  python3 run.py --trace              # 打印阶段轨迹
  python3 run.py --mcu-server         # 只起独立仿真 MCU（UDP，供 HITL）
"""
from __future__ import annotations
import argparse
import json
import os

import numpy as np


def load_cfg(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser(description="diesel 机械臂 SITL（合成/仿真级）")
    p.add_argument("--config", default=os.path.join(here, "config.json"))
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--trace", action="store_true")
    p.add_argument("--mcu-server", action="store_true", help="只起独立仿真 MCU（UDP）")
    args = p.parse_args()

    cfg = load_cfg(args.config)
    import plant as plant_mod
    import protocol as proto
    from sitl import SITL

    print("数学自检:", proto.self_test())
    import kinematics as kin
    print("运动学自检:", kin.self_test())

    if args.mcu_server:
        plant_mod.mcu_server(cfg)
        return

    rng = np.random.default_rng(cfg["sim"]["seed"])
    mcu = plant_mod.SimMCU(cfg, rng)
    transport = cfg["sim"].get("transport", "in_process")
    if transport == "udp":
        link = plant_mod.UdpLink(cfg, mcu)
    else:
        link = plant_mod.InProcessLink(mcu)

    print(f"\n=== SITL 闭环开始（mode={cfg['sim']['mode']}, transport={transport}）===")
    print("⚠ 合成/仿真级，非实机；臂协议为占位草案，待电控确认。\n")
    sitl = SITL(cfg, link)
    result = sitl.run(args.duration)

    if args.trace:
        last = None
        for row in sitl.trace:
            if row["stage"] != last:
                print(f"  t={row['t']:6.2f}s  阶段={row['stage']}")
                last = row["stage"]
    print("\n=== 结果 ===")
    for k, v in result.items():
        print(f"  {k}: {v}")
    print("\n判定:", "✓ 闭环抓取放置成功（仿真级）" if result["success"] else "✗ 未完成，需排查")


if __name__ == "__main__":
    main()
