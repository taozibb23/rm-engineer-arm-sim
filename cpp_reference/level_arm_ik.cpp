#include "level_arm_ik.hpp"

#include <cmath>

namespace diesel_vision_backend::arm {
namespace {

constexpr double kEpsR = 1e-8;
constexpr double kAcosTol = 1e-9;

// 正运动学（与战队 level_arm.cpp 一致），仅用于逆解往返校验。
Eigen::Vector3d forward_position(const LevelArmAngles& a, const LevelArmGeometry& g) {
    const double upper_r = g.upper_length_m * std::cos(a.upper_elevation);
    const double upper_z = g.upper_length_m * std::sin(a.upper_elevation);
    const double radius = upper_r + g.forearm_length_m * std::cos(a.forearm_elevation);
    const double height = upper_z + g.forearm_length_m * std::sin(a.forearm_elevation);
    const double cy = std::cos(a.base_yaw), sy = std::sin(a.base_yaw);
    return {radius * cy, radius * sy, height};
}

bool within(const std::optional<std::pair<double, double>>& lim, double v) {
    return !lim || (v >= lim->first && v <= lim->second);
}

bool allowed(const LevelArmAngles& a, const std::optional<JointLimits>& limits) {
    if (!limits) return true;
    return within(limits->base_yaw, a.base_yaw) &&
           within(limits->upper_elevation, a.upper_elevation) &&
           within(limits->forearm_elevation, a.forearm_elevation);
}

} // namespace

std::vector<LevelArmAngles> inverse_position(const Eigen::Vector3d& target_base,
                                             const LevelArmGeometry& geometry,
                                             const std::optional<JointLimits>& limits) {
    std::vector<LevelArmAngles> sols;
    if (!target_base.allFinite() || geometry.upper_length_m <= 0 || geometry.forearm_length_m <= 0)
        return sols;
    const double l1 = geometry.upper_length_m, l2 = geometry.forearm_length_m;
    const double r = std::hypot(target_base.x(), target_base.y());
    const double z = target_base.z();
    const double c2 = (r * r + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    if (c2 < -1.0 - kAcosTol || c2 > 1.0 + kAcosTol)
        return sols; // 不可达
    const double psi = r > kEpsR ? std::atan2(target_base.y(), target_base.x()) : 0.0;
    for (double sign : {-1.0, 1.0}) {
        const double theta2 = sign * std::acos(std::clamp(c2, -1.0, 1.0));
        const double alpha = std::atan2(z, r) -
                             std::atan2(l2 * std::sin(theta2), l1 + l2 * std::cos(theta2));
        LevelArmAngles a{psi, alpha, alpha + theta2};
        if (!allowed(a, limits)) continue;
        // FK 往返校验：位置必须回到目标
        if ((forward_position(a, geometry) - target_base).norm() < 1e-6)
            sols.push_back(a);
    }
    return sols;
}

} // namespace diesel_vision_backend::arm
