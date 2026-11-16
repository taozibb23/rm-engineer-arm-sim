"""运动学臂植物 + 仿真 MCU + 抓取逻辑 + 传输层。合成/仿真级，非实机。

- ArmPlant：关节一阶限速跟踪 + 夹爪开合 + 自上而下抓取/放置（无接触力/动力学）。
- SimMCU：解码控制帧 -> 驱动植物 -> 回反馈帧。假装是下位机。
- 传输：InProcessLink（纯仿真）/ UdpLink（半实物 HITL，真实电控可经 UDP 桥接入）。
"""
from __future__ import annotations
import socket
import numpy as np
import kinematics as kin
import protocol as proto


class ArmPlant:
    def __init__(self, cfg, rng):
        a = cfg["arm"]; g = cfg["grasp"]; s = cfg["scenario"]
        self.l1, self.l2 = a["l1_m"], a["l2_m"]
        self.speed = a["joint_speed_limit_rad_s"]
        self.limits = a["limits_rad"]
        self.q = np.array(s["home_angles_rad"], float)
        self.q_target = self.q.copy()
        self.gripper = 0          # 0 张开 1 闭合
        self.gripper_cmd = 0
        self.grasp_h = g["grasp_height_m"]
        self.attach_tol = g["attach_tol_m"]
        self.target_pos = np.array(s["target_pose_base"]["translation_m"], float)
        self.target_yaw = s["target_pose_base"].get("rpy_rad", [0, 0, 0])[2]
        self.install_pos = np.array(s["install_pose_base"]["translation_m"], float)
        self.attached = False
        self.placed = False
        self.status = 0

    def tool_pos(self):
        return kin.forward(self.q[0], self.q[1], self.q[2], self.l1, self.l2)["end_pos"]

    def step(self, dt):
        # 关节限速跟踪
        dq = self.q_target - self.q
        max_step = self.speed * dt
        self.q = self.q + np.clip(dq, -max_step, max_step)
        self.gripper = self.gripper_cmd
        tp = self.tool_pos()
        grasp_point = self.target_pos + np.array([0, 0, self.grasp_h])
        if self.gripper == 1 and not self.attached and not self.placed:
            if np.linalg.norm(tp - grasp_point) < self.attach_tol:
                self.attached = True
        if self.gripper == 0 and self.attached:
            self.attached = False
            self.placed = True
            self.target_pos = tp - np.array([0, 0, self.grasp_h])
        if self.attached:
            self.target_pos = tp - np.array([0, 0, self.grasp_h])
            self.target_yaw = self.q[0]
        moving = np.linalg.norm(self.q_target - self.q) > 1e-3
        # status 位标志：bit0=moving bit1=attached bit2=placed
        self.status = (1 if moving else 0) | (2 if self.attached else 0) | (4 if self.placed else 0)
        return self

    def T_base_target(self):
        return kin.se3(kin.rot_z(self.target_yaw), self.target_pos)

    def feedback(self, seq, stamp_us):
        return proto.ArmFeedback(seq, stamp_us, float(self.q[0]), float(self.q[1]),
                                 float(self.q[2]), self.gripper, self.status)


class SimMCU:
    """仿真下位机：收控制帧 -> 驱动植物 -> 发反馈帧。"""
    def __init__(self, cfg, rng):
        self.plant = ArmPlant(cfg, rng)
        self.ctrl_type = cfg["protocol"]["arm_control_msg_type"]
        self.fb_type = cfg["protocol"]["arm_feedback_msg_type"]
        self.last_seq = 0
        self.t_us = 0

    def recv_control(self, buf):
        c = proto.decode_control(buf, self.ctrl_type)
        self.last_seq = c.seq
        self.plant.q_target = np.array([c.psi, c.alpha, c.beta])
        self.plant.gripper_cmd = c.gripper

    def step(self, dt):
        self.t_us += int(dt * 1e6)
        self.plant.step(dt)

    def feedback_buf(self):
        return proto.encode_feedback(self.plant.feedback(self.last_seq, self.t_us), self.fb_type)


class InProcessLink:
    """纯仿真：PC 与仿真 MCU 同进程直连。"""
    def __init__(self, mcu: SimMCU):
        self.mcu = mcu

    def send_control(self, buf):
        self.mcu.recv_control(buf)

    def step(self, dt):
        self.mcu.step(dt)

    def recv_feedback(self):
        return self.mcu.feedback_buf()


class UdpLink:
    """半实物 HITL：PC 经 UDP 与（仿真或真实）MCU 通信。真实电控可经 UDP-串口桥接入。"""
    def __init__(self, cfg, mcu: SimMCU = None):
        u = cfg["sim"]["udp"]
        self.addr = (u["host"], u["mcu_port"])
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((u["host"], u["pc_port"]))
        self.sock.settimeout(0.5)
        self.mcu = mcu  # 若给定，则本地附带仿真 MCU（自环测试）

    def send_control(self, buf):
        self.sock.sendto(buf, self.addr)
        if self.mcu:
            self.mcu.recv_control(buf)

    def step(self, dt):
        if self.mcu:
            self.mcu.step(dt)
            self.sock.sendto(self.mcu.feedback_buf(), (self.sock.getsockname()[0], self.sock.getsockname()[1]))

    def recv_feedback(self):
        try:
            buf, _ = self.sock.recvfrom(256)
            return buf
        except socket.timeout:
            return None


def mcu_server(cfg):
    """独立仿真 MCU 进程入口（UDP），供 HITL 用。"""
    import json
    rng = np.random.default_rng(cfg["sim"]["seed"])
    mcu = SimMCU(cfg, rng)
    u = cfg["sim"]["udp"]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((u["host"], u["mcu_port"]))
    print(f"[sim MCU] 监听 {u['host']}:{u['mcu_port']}（占位臂协议，待电控确认）")
    dt = cfg["sim"]["dt_s"]
    while True:
        sock.settimeout(dt)
        try:
            buf, addr = sock.recvfrom(256)
            mcu.recv_control(buf)
        except socket.timeout:
            pass
        mcu.step(dt)
        sock.sendto(mcu.feedback_buf(), addr if 'addr' in dir() else (u["host"], u["pc_port"]))
