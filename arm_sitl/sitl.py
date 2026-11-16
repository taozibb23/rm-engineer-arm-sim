"""SITL 闭环编排 + 抓取任务状态机。合成/仿真级，非实机。

闭环：反馈q → 仿真摄像头 → 坐标链估计目标 → 任务状态机 → IK → 协议帧 → 仿真MCU → 植物 → 反馈。
任务用**视觉估计**的目标位姿（不是真值），这才是闭环的意义。
"""
from __future__ import annotations
import numpy as np
import kinematics as kin
import protocol as proto
from camera import CameraSim, target_in_base

STAGES = ["observe", "pregrasp", "approach", "grasp", "lift", "transfer", "place", "release", "done"]


class Task:
    def __init__(self, cfg):
        g = cfg["grasp"]; s = cfg["scenario"]
        self.pregrasp_h = g["pregrasp_height_m"]
        self.grasp_h = g["grasp_height_m"]
        self.lift_h = g["lift_height_m"]
        self.install_pos = np.array(s["install_pose_base"]["translation_m"], float)
        self.home = np.array(s["home_angles_rad"], float)
        self.stage = "observe"
        self.gripper = 0
        self.observe_frames = 0
        self.tool_target = None
        self.lift_ref = None
        self.reach_tol = 0.006

    def _above(self, p, h):
        return np.array([p[0], p[1], p[2] + h])

    def update(self, est_target_pos, q_actual, status):
        """返回 (tool_target_pos, gripper_cmd)。est_target_pos 可能 None（丢帧）。
        注意：每阶段先设本阶段 tool_target，再判到位，避免用上一阶段旧目标误判。"""
        attached = bool(status & 2)
        placed = bool(status & 4)
        tp = kin.forward(q_actual[0], q_actual[1], q_actual[2])["end_pos"]

        def reached():
            return self.tool_target is not None and np.linalg.norm(tp - self.tool_target) < self.reach_tol

        if self.stage == "observe":
            self.observe_frames += 1
            if est_target_pos is not None and self.observe_frames >= 10:
                self.stage = "pregrasp"
            return (self._above(est_target_pos, self.pregrasp_h) if est_target_pos is not None else None), 0

        if self.stage == "pregrasp":
            if est_target_pos is None:  # 丢帧：保持上一目标
                return self.tool_target, 0
            self.tool_target = self._above(est_target_pos, self.pregrasp_h)
            if reached():
                self.stage = "approach"
            return self.tool_target, 0

        if self.stage == "approach":
            if est_target_pos is None:
                return self.tool_target, 0
            self.tool_target = self._above(est_target_pos, self.grasp_h)
            if reached():
                self.stage = "grasp"
            return self.tool_target, 0

        if self.stage == "grasp":
            self.gripper = 1
            if attached:
                # 快照固定参考，避免夹住后目标随末端移动造成"移动球门"
                self.lift_ref = est_target_pos.copy() if est_target_pos is not None else (self.tool_target - np.array([0, 0, self.grasp_h]))
                self.stage = "lift"
            return self.tool_target, 1

        if self.stage == "lift":
            self.tool_target = self._above(self.lift_ref, self.lift_h)
            if reached():
                self.stage = "transfer"
            return self.tool_target, 1

        if self.stage == "transfer":
            self.tool_target = self._above(self.install_pos, self.pregrasp_h)
            if reached():
                self.stage = "place"
            return self.tool_target, 1

        if self.stage == "place":
            self.tool_target = self._above(self.install_pos, self.grasp_h)
            if reached():
                self.stage = "release"
            return self.tool_target, 1

        if self.stage == "release":
            self.gripper = 0
            if placed:
                self.stage = "done"
            return self.tool_target, 0

        return None, 0  # done


class SITL:
    def __init__(self, cfg, link):
        self.cfg = cfg
        self.link = link
        self.rng = np.random.default_rng(cfg["sim"]["seed"])
        self.l1, self.l2 = cfg["arm"]["l1_m"], cfg["arm"]["l2_m"]
        self.limits = cfg["arm"]["limits_rad"]
        self.cam = CameraSim(cfg, self.rng)
        self.task = Task(cfg)
        self.dt = cfg["sim"]["dt_s"]
        self.ctrl_type = cfg["protocol"]["arm_control_msg_type"]
        self.fb_type = cfg["protocol"]["arm_feedback_msg_type"]
        self.q_actual = np.array(cfg["scenario"]["home_angles_rad"], float)
        self.seq = 0
        self.t = 0.0
        self.trace = []

    def step(self):
        # 1. 反馈
        fb_buf = self.link.recv_feedback()
        status = 0
        if fb_buf:
            fb = proto.decode_feedback(fb_buf, self.fb_type)
            self.q_actual = np.array([fb.psi, fb.alpha, fb.beta])
            status = fb.status
        # 2. 仿真摄像头（用植物真值目标 + 当前关节）
        T_true = self.link.mcu.plant.T_base_target() if hasattr(self.link, "mcu") and self.link.mcu else None
        obs = self.cam.observe(self.q_actual, T_true, self.t, self.l1, self.l2) if T_true is not None else {"valid": False, "stamp_s": self.t}
        # 3. 坐标链估计
        T_est = target_in_base(obs, self.q_actual, self.t, self.cam, self.l1, self.l2)
        est_pos = T_est[:3, 3] if T_est is not None else None
        # 4. 任务 -> 工具目标
        tool_target, grip = self.task.update(est_pos, self.q_actual, status)
        # 5. IK
        q_target = self.q_actual
        if tool_target is not None:
            sol = kin.pick_closest(kin.inverse(tool_target, self.l1, self.l2, self.limits), self.q_actual)
            if sol:
                q_target = kin.q_to_vec(sol)
        # 6. 协议帧 -> MCU
        self.seq += 1
        ctrl = proto.ArmControl(self.seq, int(self.t * 1e6), float(q_target[0]), float(q_target[1]), float(q_target[2]), grip, 0)
        self.link.send_control(proto.encode_control(ctrl, self.ctrl_type))
        # 7. 植物步进
        self.link.step(self.dt)
        self.t += self.dt
        self.trace.append({"t": self.t, "stage": self.task.stage, "est": est_pos, "status": status})

    def run(self, duration_s=None, verbose=True):
        duration = duration_s or self.cfg["sim"]["duration_s"]
        n = int(duration / self.dt)
        for _ in range(n):
            self.step()
            if self.task.stage == "done":
                break
        return self.summary()

    def summary(self):
        plant = self.link.mcu.plant if hasattr(self.link, "mcu") and self.link.mcu else None
        placed_err = None
        if plant is not None:
            placed_err = float(np.linalg.norm(plant.target_pos - self.task.install_pos))
        return {
            "stage": self.task.stage,
            "frames": len(self.trace),
            "sim_time_s": round(self.t, 3),
            "attached": bool(plant.attached) if plant else None,
            "placed": bool(plant.placed) if plant else None,
            "place_error_m": placed_err,
            "success": self.task.stage == "done" and (plant.placed if plant else False),
        }
