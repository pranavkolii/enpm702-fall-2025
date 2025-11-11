/**
 * @file main.cpp
 * @author Pranav Jagdish Koli
 * @brief The main cpp file for this assignment where all the functions, structs, etc. are utilized.
 * @version 0.1
 * @date 2025-11-5
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "robot_types.hpp"
#include "robot_kinematics.hpp"
#include "robot_control.hpp"

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <iomanip>

int main() {
    std::cout << "=== Robot Kinematics & Control ===\n\n";

    // 1) Start/Goal 
    // Aggregate Initialization
    const JointState start{0.0, 0.0};                 // θ1=0, θ2=0
    const JointState goal{M_PI / 4.0, -M_PI / 6.0};   // θ1=45°, θ2=-30°

    std::cout << "Checking the joint angles: \n";
    bool joint1_check = check_joint_limits(start);
    bool joint2_check = check_joint_limits(goal);

    if (joint1_check && joint2_check){
        std::cout << "Both joints are within the mechanical constraints. \n\n";
    } else {
        std::cout << "Joint angles are out of the mechanical constraints. \n\n";
    }

    std::cout << "Generating smooth trajectory between:\n";
    std::cout << "Start  -> ";
    print_joint_state(start);
    std::cout << "Goal   -> ";
    print_joint_state(goal);
    std::cout << '\n';

    // Task 4: Trajectory Management with Smart Pointers.
    // 2) Trajectory container (unique ownership )
    /**
     * @brief Creating a unique pointer traj to hold the trajectory points for joint angles. 
     * 
     */
    auto traj = std::make_unique<std::vector<JointState>>();
    traj->reserve(k_num_samples);
    std::cout << "Trajectory points: " << k_num_samples << '\n';

    // Task 3: Linear Trajectory and Velocity Filtering
    for (int i{0} ; i < k_num_samples; ++i) {
        const double alpha = static_cast<double>(i) * k_alpha_step;
        traj->push_back(interpolate_linear(start, goal, alpha));
    }

    std::cout << "Unfiltered Trajectory (every 5th point shown): \n";
    print_trajectory(*traj, 5, k_num_samples);     // Printing the trajectory for every 5th point within the sample limit.
    
    // Apply velocity-limit filter (lambda)
    std::cout << "\nApplying velocity-limit filter: |dθ| ≤ " << k_vel_limit << " rad/s\n";

    /**
     * @brief A lambda function for clamping the velocity to the limit.
     * 
     */
    auto clamp_to_limit = [](double v, double limit) -> double {
            if (v > limit) {
                return limit;
            }
            if (v < -limit) {
                return -limit;
            }
            return v; 
        };
    
    /**
     * @brief A lambda function utilizing the earlier clamping function on the current joint velocities.
     * 
     */
    auto clamp_vel = [&clamp_to_limit](const JointState& s) -> JointState {
        JointState out{s};

        out.dtheta1 = clamp_to_limit(out.dtheta1, k_vel_limit);
        out.dtheta2 = clamp_to_limit(out.dtheta2, k_vel_limit);

        return out;
    };
    
    apply_filter(*traj, clamp_vel);
    
    std::cout << "-> Filter applied succesfully, all values within limits. \n\n";
    std::cout << "Filtered Trajectory (first 5 points): \n";
    print_trajectory(*traj, 1, 5);     // Printing the trajectory for every nth point for the limit of 5 sample.

 
    // 4) End-effector poses (unique ownership)
    auto ee_poses = std::make_unique<std::vector<EndEffectorPose>>();
    ee_poses->reserve(traj->size());

    for (const auto& j : *traj) {
        ee_poses->push_back(forward_kinematics(j));
    }

    std::cout << "\nComputing end-effector poses for filtered trajectory...\n";
    std::cout << std::fixed << std::setprecision(2) << "Link lengths: L1 = " << k_link1 << " m, L2 = " << k_link2 << " m \n\n";

    std::cout << "End-Effector Trajectory (all points):\n";
    print_ee_poses(*ee_poses,k_num_samples);


    // Reporting all the robot related data.
   
    std::cout << "\n\nSummary\n" << "--------\n";
    std::cout << "* Total joint states: " << k_num_samples << '\n';
    std::cout << "* Velocity filter: active (|dθ| ≤ " << k_vel_limit << ")\n\n";
    std::cout << "\nProgram finished successfully.\n";
}

// All smart pointer go out of scope.
// So all resources are released automatically (RAII).