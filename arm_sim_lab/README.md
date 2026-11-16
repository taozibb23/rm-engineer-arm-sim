# arm_sim_lab — 运动学手算核对 + 出图

> 合成 / 仿真级，非实机。Python + numpy（出图需 matplotlib）。

三自由度机械臂（`ψ`/`α`/`β`，`L1=L2=0.149m`，末端水平）的运动学学习/验证工具。
**手算 → 它核对 + 画出来**。数学核心 `arm_kinematics.py` 对齐战队 C++ `level_arm.cpp`，零 ROS。

## 用法

```bash
python3 arm_kinematics.py            # 核心自检（对齐 C++）
python3 arm_sim.py --verify          # 核对 3 道手算练习 + C++ 用例
python3 arm_sim.py --fk 0 90 0       # 正解：ψ α β（度）
python3 arm_sim.py --ik 0.149 0 0.149  # 逆解：目标 xyz（米），画肘上/肘下两解
python3 arm_sim.py --workspace       # 可达 workspace
python3 arm_sim.py --interactive     # 滑条拖 ψ/α/β
# 加 --save out.png 只存图；加 --no-show 只算不画
```

## 核心一课：多解

目标 `(0.149, 0, 0.149)` 有两组关节角 `(α=90°,β=0°)` 与 `(α=0°,β=90°)`——
同一点、不同关节角。这就是逆解必须配**限位 + 选解策略**的原因。

## 约定

- `α`、`β` 相对水平面的仰角，`β` 不是肘部相对夹角；内部 rad，界面度。
- 输出是小臂末端**轴点**，非抓取中心（夹爪偏置未测）。
- 几何角 ≠ 电机编码器角；传动映射未确认前禁止把几何角当电机角下发。

## 边界

轴心距/轴偏置/限位未实测；无碰撞/动力学/夹爪；不是物理仿真。
实物 IK 的 C++ 实现仍需本人编写，本工具是参考/验证。
