/**
 * @file robot_types.hpp
 * @author Pranav Jagdish Koli
 * @brief A robot types header file consisting of declaration of structs for state of the robot, and values of certain paramters,
 * @version 0.1
 * @date 2025-11-5
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once
#include <iostream>
#include <vector>

// Task 1: Robot State POD Structs
/**
 * @brief stores the current state of the joints in terms of joint angles & joint velocities.
 * 
 */
struct JointState {
    double theta1;        // Joint 1 angle [rad]
    double theta2;        // Joint 2 angle [rad]
    // Default member initializers for velocities
    double dtheta1{0.0}; // Joint 1 velocity [rad/s]
    double dtheta2{0.0}; // Joint 2 velocity [rad/s]
};

/**
 * @brief Store the end effector position in terms of x & y (in meters) coordinates. 
 * 
 */
struct EndEffectorPose {
    double x; // [m]
    double y; // [m]
};

// Declaring print function for Joinstate, trajectory points and End Effector poses (non-template -> implemented in .cpp)
void print_joint_state(const JointState& js);
void print_trajectory(const std::vector<JointState>& trajectory, const int nth_point, size_t limit);
void print_ee_poses(const std::vector<EndEffectorPose>& ee_trajectory, size_t ee_limit);

inline constexpr double k_link1{0.5};      // [m]
inline constexpr double k_link2{0.3};     // [m]
inline constexpr double k_vel_limit{1.0};   // [rad/s]
inline constexpr int    k_num_samples{21};  // includes endpoints
inline constexpr double k_alpha_step{1.0 / (k_num_samples - 1)};
