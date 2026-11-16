// 三自由度机械臂位置逆解参考测试（合成/仿真级）。
// 编译（需 Eigen3）：
//   g++ -std=c++17 -O2 -I/usr/include/eigen3 test_level_arm_ik.cpp level_arm_ik.cpp -o test_ik && ./test_ik
#include "level_arm_ik.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
    using namespace diesel_vision_backend::arm;
    const auto require = [](bool ok, const char* msg) {
        if (!ok) throw std::runtime_error(msg);
    };
    const double pi = std::acos(-1.0);
    LevelArmGeometry g;

    // 1) 伸直边界 (0.298,0,0)：c2=1，alpha=beta=0
    auto s1 = inverse_position({0.298, 0, 0}, g);
    require(!s1.empty(), "straight should have solution");
    require(std::abs(s1[0].upper_elevation) < 1e-9 && std::abs(s1[0].forearm_elevation) < 1e-9,
            "straight alpha=beta=0");

    // 2) 多解 (0.149,0,0.149)：肘上/肘下两组，往返一致
    auto s2 = inverse_position({0.149, 0, 0.149}, g);
    require(s2.size() == 2, "should have 2 solutions");
    for (const auto& a : s2) {
        const double ur = g.upper_length_m * std::cos(a.upper_elevation);
        const double uz = g.upper_length_m * std::sin(a.upper_elevation);
        const double r = ur + g.forearm_length_m * std::cos(a.forearm_elevation);
        const double h = uz + g.forearm_length_m * std::sin(a.forearm_elevation);
        require(std::abs(r - 0.149) < 1e-6 && std::abs(h - 0.149) < 1e-6, "round trip");
    }

    // 3) 不可达 (0.5,0,0)：无解
    require(inverse_position({0.5, 0, 0}, g).empty(), "unreachable should be empty");

    // 4) yaw 正确：目标 (0,0.2,0.1) → psi≈90°
    auto s4 = inverse_position({0, 0.2, 0.1}, g);
    require(!s4.empty(), "yaw target reachable");
    require(std::abs(s4[0].base_yaw - pi / 2) < 1e-6, "psi=90deg");

    // 5) 限位过滤：限制大臂仰角 < 0.1，则 (0.149,0,0.149) 的 90° 解被滤掉
    JointLimits lim;
    lim.upper_elevation = std::make_pair(-0.1, 0.1);
    auto s5 = inverse_position({0.149, 0, 0.149}, g, lim);
    for (const auto& a : s5) require(a.upper_elevation <= 0.1, "limit enforced");

    // 6) 非法输入：NaN 目标 → 无解
    require(inverse_position({std::nan(""), 0, 0}, g).empty(), "NaN rejected");

    std::cout << "PASS: 伸直边界 / 多解往返 / 不可达 / yaw / 限位 / NaN\n";
    return 0;
}
