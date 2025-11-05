/**
 * @file robot_types.cpp
 * @author Pranav Jagdish Koli
 * @brief 
 * @version 0.1
 * @date 2025-10-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "robot_types.hpp"
#include <iomanip>
#include <vector>

void print_joint_state(const JointState& js) {
    std::cout << std::fixed << std::setprecision(4)
              << "θ1 = " << js.theta1 << " rad | "
              << "θ2 = " << js.theta2 << " rad | "
              << "dθ1 = " << js.dtheta1 << " rad/s | "
              << "dθ2 = " << js.dtheta2 << " rad/s\n";
}

void print_trajectory(const std::vector<JointState>& trajectory, const int nth_point, size_t point_limit) {
    std::cout << std::fixed << std::setprecision(4);

    for (size_t i = 0; i < trajectory.size(); i += nth_point) {
        const auto& s = trajectory[i];
        if ((i < point_limit) && (i <= trajectory.size() - 1)) {
            std::cout << "[" << i << "] θ1 = " << s.theta1 << " rad | θ2 = " << s.theta2 << " rad | ";            
            std::cout << "dθ1 = " << s.dtheta1 << " rad/s | dθ2 = " << s.dtheta2 << " rad/s\n";
        }
    }
}

void print_ee_poses(const std::vector<EndEffectorPose>& ee_trajectory, size_t ee_limit) {
    std::cout << std::fixed << std::setprecision(4);

    for (size_t j = 0; j < ee_trajectory.size(); ++j) {
        const auto& s = ee_trajectory[j];
        if ((j < ee_limit) && (j <= ee_trajectory.size() - 1)) {
            std::cout << "[" << j << "] x = " << s.x << " m | y = " << s.y << " m\n";            
        }
    }
}