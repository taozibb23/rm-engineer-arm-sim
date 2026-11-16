"""仿真摄像头（合成观测注入）+ 手眼坐标链。合成/仿真级，非实机。

数据流（eye-in-hand）：
  T_base_camera(q) = T_base_tool(q) · T_tool_camera        （手眼，随关节变）
  T_camera_target  = inv(T_base_camera) · T_base_target_true
  观测 = T_camera_target + 噪声 / 丢帧
后端坐标链（对齐 observation.cpp::target_in_base）：
  T_base_target_est = T_base_camera(q_actual) · T_camera_target_obs
"""
from __future__ import annotations
import numpy as np
import kinematics as kin


class CameraSim:
    def __init__(self, cfg, rng):
        c = cfg["camera"]
        self.T_tool_camera = kin.pose_from_cfg(c["T_tool_camera"])
        self.pos_sigma = c["noise"]["pos_sigma_m"]
        self.rot_sigma = c["noise"]["rot_sigma_rad"]
        self.dropout = c["noise"]["dropout_prob"]
        self.max_age = c["max_age_s"]
        self.max_pairing = c["max_pairing_diff_s"]
        self.rng = rng

    def T_base_camera(self, q, l1, l2):
        return kin.T_base_tool(q, l1, l2) @ self.T_tool_camera

    def observe(self, q, T_base_target_true, stamp_s, l1, l2):
        """仿真一帧：返回相机系目标观测 dict（可能 valid=False 表示丢帧）。"""
        if self.rng.random() < self.dropout:
            return {"valid": False, "stamp_s": stamp_s}
        T_cam_target = kin.inv_se3(self.T_base_camera(q, l1, l2)) @ T_base_target_true
        t = T_cam_target[:3, 3] + self.rng.normal(0, self.pos_sigma, 3)
        # 小角度旋转噪声
        d = self.rng.normal(0, self.rot_sigma, 3)
        dR = kin.rot_rpy(d)
        R = dR @ T_cam_target[:3, :3]
        return {
            "valid": True, "stamp_s": stamp_s,
            "R_camera_target": R, "t_camera_target": t,
            "confidence": 0.9, "reprojection_error_px": 0.5,
        }


def target_in_base(obs, q_actual, stamp_now, cam: CameraSim, l1, l2):
    """后端坐标链 + 时间配对（对齐 observation.cpp）。失败返回 None，不伪造零位姿。"""
    if not obs.get("valid"):
        return None
    ts = obs["stamp_s"]
    if not np.isfinite(ts) or ts < 0 or stamp_now < ts:
        return None
    if stamp_now - ts > cam.max_age:
        return None
    T_base_camera = cam.T_base_camera(q_actual, l1, l2)
    T_cam_target = kin.se3(obs["R_camera_target"], obs["t_camera_target"])
    if obs["t_camera_target"][2] <= 0:  # 相机系深度必须为正
        return None
    return T_base_camera @ T_cam_target
