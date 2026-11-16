"""SITL 平台测试（合成/仿真级）。python3 test_sitl.py"""
from __future__ import annotations
import copy
import json
import os
import numpy as np

import kinematics as kin
import protocol as proto
import plant as plant_mod
from camera import CameraSim, target_in_base
from sitl import SITL

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = json.load(open(os.path.join(HERE, "config.json"), encoding="utf-8"))


def cfg_with(**over):
    c = copy.deepcopy(BASE)
    for path, val in over.items():
        keys = path.split(".")
        d = c
        for k in keys[:-1]:
            d = d[k]
        d[keys[-1]] = val
    return c


def build_link(cfg):
    rng = np.random.default_rng(cfg["sim"]["seed"])
    mcu = plant_mod.SimMCU(cfg, rng)
    return plant_mod.InProcessLink(mcu)


def t_kinematics():
    print(kin.self_test())


def t_protocol():
    print(proto.self_test())


def t_coord_chain_zero_noise():
    cfg = cfg_with(**{"camera.noise.pos_sigma_m": 0.0, "camera.noise.rot_sigma_rad": 0.0})
    rng = np.random.default_rng(0)
    cam = CameraSim(cfg, rng)
    q = np.array([0.3, 0.4, 0.2])
    T_true = kin.se3(kin.rot_z(0.2), np.array([0.18, 0.05, 0.03]))
    obs = cam.observe(q, T_true, 1.0, cfg["arm"]["l1_m"], cfg["arm"]["l2_m"])
    T_est = target_in_base(obs, q, 1.0, cam, cfg["arm"]["l1_m"], cfg["arm"]["l2_m"])
    err = np.linalg.norm(T_est[:3, 3] - T_true[:3, 3])
    assert err < 1e-9, err
    print(f"PASS: 零噪声坐标链往返误差 {err:.2e} m")


def t_closed_loop():
    cfg = cfg_with()
    s = SITL(cfg, build_link(cfg))
    r = s.run()
    assert r["success"], r
    assert r["placed"] and r["place_error_m"] < 0.005, r
    print(f"PASS: 闭环抓取放置成功，放置误差 {r['place_error_m']:.2e} m，{r['frames']} 帧")


def t_noise_robust():
    cfg = cfg_with(**{"camera.noise.pos_sigma_m": 0.003, "camera.noise.rot_sigma_rad": 0.01})
    s = SITL(cfg, build_link(cfg))
    r = s.run()
    assert r["success"], r
    assert r["place_error_m"] < 0.02, r
    print(f"PASS: 带噪声闭环仍成功，放置误差 {r['place_error_m']:.4f} m")


def t_dropout():
    cfg = cfg_with(**{"camera.noise.dropout_prob": 0.2})
    s = SITL(cfg, build_link(cfg))
    r = s.run(duration_s=40)
    assert r["success"], r
    print(f"PASS: 20% 丢帧闭环仍成功，{r['frames']} 帧")


def t_unreachable_no_crash():
    # 目标放可达范围外：IK 无解，任务不应崩溃
    cfg = cfg_with(**{"scenario.target_pose_base.translation_m": [0.5, 0.0, 0.05]})
    s = SITL(cfg, build_link(cfg))
    r = s.run(duration_s=5)
    assert not r["success"]
    print("PASS: 不可达目标不崩溃（无解时保持原位）")


if __name__ == "__main__":
    for fn in [t_kinematics, t_protocol, t_coord_chain_zero_noise, t_closed_loop,
               t_noise_robust, t_dropout, t_unreachable_no_crash]:
        fn()
    print("\n全部 SITL 测试通过 ✓（合成/仿真级，非实机）")
