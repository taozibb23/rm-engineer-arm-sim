# arm_sitl — 软件在环仿真平台（SITL）

> 合成 / 仿真级，非实机。理想运动学 + 合成观测 + 占位协议。
> 抓取/放置成功是仿真结论；硬件/手眼/协议参数实测后改 `config.json` 即可适配。

## 闭环

```
 仿真摄像头 ──相机系观测──► 坐标链估计目标 ──► 任务状态机 ──► IK
      ▲                                                  │
      │                                                  ▼
 手眼(随关节) ◄──关节反馈── 运动学臂+抓取 ◄── 仿真MCU ◄── 协议适配层(RoboLink风格)
```

任务用**视觉估计**的目标位姿（不是真值）驱动——这才是闭环的意义。

## 三种接入模式

| 模式 | 怎么跑 | 状态 |
|---|---|---|
| 纯仿真 | `python3 run.py`（仿真 MCU 同进程） | ✅ 已跑通 |
| 半实物 HITL | `python3 run.py --mcu-server` 起仿真 MCU(UDP)，PC 端 `transport=udp`；真实电控可经 UDP-串口桥接入 | ✅ UDP 自环已验证 |
| 全实物 | 接真臂（硬件就绪后） | ⏳ 待硬件 + 电控协议 |

## 文件

| 文件 | 职责 |
|---|---|
| `config.json` | 所有硬件/相机/手眼/协议/仿真参数，改这里适配真机 |
| `kinematics.py` | FK/IK + SE(3)，对齐 C++ `level_arm.cpp` |
| `camera.py` | 仿真摄像头（合成观测+噪声/丢帧）+ 手眼坐标链 |
| `protocol.py` | 占位臂协议：帧/CRC32/编解码（待电控确认） |
| `plant.py` | 运动学臂植物 + 仿真 MCU + 抓取逻辑 + 传输（InProcess/UDP） |
| `sitl.py` | 闭环编排 + 抓取任务状态机 |
| `run.py` / `test_sitl.py` | 入口 / 测试 |

## 怎么跑

```bash
python3 run.py --trace        # 纯仿真闭环，打印阶段轨迹
python3 test_sitl.py          # 全部测试
python3 run.py --mcu-server   # 独立仿真 MCU（UDP，HITL 用）
python3 run.py --config 你的.json
```

## 已验证（test_sitl.py 全过）

- 运动学对齐 C++；协议编解码 + CRC 篡改拒绝。
- 零噪声坐标链往返误差 ~7e-18 m；闭环放置误差 ~6e-9 m。
- 带噪声（pos 3mm/rot 0.01rad）放置误差 0.0000 m；20% 丢帧仍成功；不可达不崩溃。
- InProcess 与 UDP 两种传输都跑通。

## ⚠ 接真实电控前的硬依赖（P0）

1. **臂协议未定义**：现有 RoboLink 只有云台帧，没有臂关节/夹爪帧。`protocol.py` 的
   msg_type 与载荷是占位草案，非正式分配。需电控/组长定臂控制+反馈帧后对账。
2. **角度域**：当前 `geometric_placeholder`（几何角）。接真实电机前必须改电机角并标定传动比/零偏。
3. **手眼外参 / 杆长 / 限位 / 轴偏置**均为占位，待实测。

## 边界

无接触力/动力学/碰撞，不是物理仿真；摄像头是合成观测注入，非渲染图像跑真实检测器；
无 URDF/CAD，未接 Gazebo/MuJoCo/MoveIt；实物 IK 的 C++ 实现仍需本人编写。
