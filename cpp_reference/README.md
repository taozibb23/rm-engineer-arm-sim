# cpp_reference — 实物 IK 的 C++ 参考实现（学习用）

> ⚠ **这是参考脚手架，不是你的贡献本身。** 按你的规矩"关键 C++ 本人读改调验"：
> 先读懂推导 → 手算核对 → 跑测试 → 再 port 进战队 `diesel_ws`，那才是你的真实贡献。
> 合成/仿真级；几何角 ≠ 电机角，传动映射未确认前禁止下发几何角。

## 文件

| 文件 | 内容 |
|---|---|
| `level_arm_ik.hpp` | 位置逆解声明 + 角度/几何/限位结构（对齐战队 `arm` 命名空间） |
| `level_arm_ik.cpp` | 两连杆余弦定理逆解 + 多解 + 可达 + 限位 + FK 往返校验 |
| `test_level_arm_ik.cpp` | 6 项测试：伸直边界/多解往返/不可达/yaw/限位/NaN |

## 编译运行（已验证通过）

```bash
g++ -std=c++17 -O2 -I/usr/include/eigen3 test_level_arm_ik.cpp level_arm_ik.cpp -o test_ik && ./test_ik
# 输出：PASS: 伸直边界 / 多解往返 / 不可达 / yaw / 限位 / NaN
```

## 核心推导（务必能手动推）

引入肘部相对角 `θ2 = β − α`，化标准两连杆：
```
r = hypot(x,y),  z = z
c2 = (r² + z² − L1² − L2²) / (2·L1·L2)     余弦定理
θ2 = ±acos(c2)                              ± → 肘上/肘下两解
α  = atan2(z,r) − atan2(L2·sinθ2, L1 + L2·cosθ2)
β  = α + θ2,  ψ = atan2(y,x)
可达：−1 ≤ c2 ≤ 1
```

## 怎么 port 进 diesel_ws（你的贡献步骤）

1. **读懂**：对照 `arm_sitl/kinematics.py` 和本文件，手推一遍逆解。
2. **核对**：用 `arm_sim_lab --verify` 手算 3 题，确认理解。
3. **整合**：把 `inverse_position` 作为方法加进战队 `LevelArmModel`（`diesel_vision_backend/arm/level_arm.hpp/.cpp`），
   复用战队已有的 `forward` 做往返校验，不要带本参考的自包含副本。
4. **测试**：把本测试用例迁进战队 `test/test_arm_geometry.cpp` 风格，`colcon build` + CTest 跑过。
5. **边界**：限位/传动映射待实测；未确认前不接真实电机。Git 提交/PR 由本人操作。

## 与仿真工具的关系

本参考的数学与 `arm_sim_lab` / `arm_sim_web` / `arm_sitl` 完全同源，都对齐战队 C++ 正运动学。
仿真工具帮你理解/验证，本参考帮你落到战队 C++——两者配合，把"看懂"变成"真实代码贡献"。
