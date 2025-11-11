/**
 * @file robot_types.cpp
 * @author Pranav Jagdish Koli
 * @brief The file consists of print functions for robot's joint state, trajectory and end effector positions.
 * @version 0.1
 * @date 2025-11-5
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "robot_types.hpp"
#include <iomanip>
#include <vector>

/**
 * @brief Function for printing the joint state of the robot
 * 
 * @param js    // A constant reference to the JointState structure
 */
void print_joint_state(const JointState& js) {
    std::cout << std::fixed << std::setprecision(4)
              << "θ1 = " << js.theta1 << " rad | "
              << "θ2 = " << js.theta2 << " rad | "
              << "dθ1 = " << js.dtheta1 << " rad/s | "
              << "dθ2 = " << js.dtheta2 << " rad/s\n";
}

/**
 * @brief Print the joint state trajectory with the limit for printing x number of states.
 * 
 * @param trajectory    // A constant reference to the trajectory vector of JointState Struct.
 * @param nth_point     // Printing the nth point (i.e. every nth point)
 * @param point_limit   // The number of points to be printed.
 */
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

/**
 * @brief Print the end effector position of the robot
 * 
 * @param ee_trajectory     // A constanct reference to the trajectory containing End Effector struct.
 * @param ee_limit          // The number of points to be printed.
 */
void print_ee_poses(const std::vector<EndEffectorPose>& ee_trajectory, size_t ee_limit) {
    std::cout << std::fixed << std::setprecision(4);

    for (size_t j = 0; j < ee_trajectory.size(); ++j) {
        const auto& s = ee_trajectory[j];
        if ((j < ee_limit) && (j <= ee_trajectory.size() - 1)) {
            std::cout << "[" << j << "] x = " << s.x << " m | y = " << s.y << " m\n";            
        }
    }
}