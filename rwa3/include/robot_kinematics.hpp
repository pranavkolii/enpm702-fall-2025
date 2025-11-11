/**
 * @file robot_kinematics.hpp
 * @author Pranav Jagdish Koli
 * @brief This file contains the templates for forward kinematics and joint angle limiter for the robot.
 * @version 0.1
 * @date 2025-11-5
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once
#include "robot_types.hpp"
#include <cmath>

// Task 2: Forward Kinematics Template
/**
 * @brief A template function used to do forward kinematics and generate the x & y coordinates of end effector
 * 
 * @tparam State 
 * @param s     // A constant reference to the state structure having joint angles.
 * @param L1    // Default value for the length of joint 1.
 * @param L2    // Default value for the length of joint 2.
 * @return EndEffectorPose 
 */
template <typename State>
EndEffectorPose forward_kinematics(const State& s,
                                   double L1 = k_link1,
                                   double L2 = k_link2)
{
    EndEffectorPose pose{};
   
    pose.x = (L1 * std::cos(s.theta1)) + (L2 * std::cos(s.theta1 + s.theta2));      // x = L1*cos(theta1) + L2*cos(theta1 + theta2)
    pose.y = (L1 * std::sin(s.theta1)) + (L2 * std::sin(s.theta1 + s.theta2));      // y = L1*sin(theta1) + L2*sin(theta1 + theta2)
    return pose;
}

// Optional Task 5: Joint Limit Validation
/**
 * @brief A template function used to validate whether the joint angles are in the range of mechanical constraints,
 * 
 * @tparam State 
 * @param s     // A constant reference to the state structure having joint angles.
 * @return true 
 * @return false 
 */

template <typename State>
bool check_joint_limits(const State& s) {
    return (((s.theta1 > -M_PI) && (s.theta1 < M_PI)) && ((s.theta2 > -M_PI) && (s.theta2 < M_PI)));
}

