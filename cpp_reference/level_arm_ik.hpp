#pragma once
// 三自由度水平末端机械臂 —— 位置逆解参考实现（合成/仿真级）。
// 风格对齐战队 diesel_vision_backend::arm/level_arm.hpp。
// ⚠ 这是学习参考：本人须读懂推导、手算核对、跑测试验证后，再 port 进 diesel_ws。
// 几何角 != 电机编码器角；传动映射未确认前禁止把几何角当电机角下发。
#include <Eigen/Geometry>
#include <optional>
#include <vector>

namespace diesel_vision_backend::arm {

// 复用战队 level_arm.hpp 的角度/几何定义（此处为参考自包含副本）。
struct LevelArmAngles {
    double base_yaw = 0.0;
    double upper_elevation = 0.0;   // 大臂相对水平仰角
    double forearm_elevation = 0.0; // 小臂相对水平仰角，非肘部夹角
};

struct LevelArmGeometry {
    double upper_length_m = 0.149;
    double forearm_length_m = 0.149;
};

// 关节限位（rad）。实物未测，默认空表示不施加。
struct JointLimits {
    std::optional<std::pair<double, double>> base_yaw;
    std::optional<std::pair<double, double>> upper_elevation;
    std::optional<std::pair<double, double>> forearm_elevation;
};

// 位置逆解（末端保持水平）：返回 0/1/2 个解（肘上/肘下），无解返回空。
// 推导：引入肘部相对角 theta2 = beta - alpha，余弦定理求 theta2=±acos(c2)，
//       alpha = atan2(z,r) - atan2(L2 sinθ2, L1 + L2 cosθ2)，beta = alpha + theta2。
// 三自由度只能定位置 + 水平姿态，不能任意满足六维姿态。
std::vector<LevelArmAngles> inverse_position(const Eigen::Vector3d& target_base,
                                             const LevelArmGeometry& geometry = {},
                                             const std::optional<JointLimits>& limits = std::nullopt);

} // namespace diesel_vision_backend::arm
