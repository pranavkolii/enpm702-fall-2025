/**
 * @file robot_control.hpp
 * @author Pranav Jagdish Koli
 * @brief The robot control header file consisting of the interpolation template & declaration of velocity limit filter.
 * @version 0.1
 * @date 2025-11-5
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once
#include "robot_types.hpp"
#include <vector>
#include <functional>
 

/**
 * @brief To interpolate state of the robot between start and goal
 * 
 * @tparam State 
 * @param start 
 * @param goal 
 * @param alpha 
 * @return State 
 */
template <typename State>
State interpolate_linear(const State& start, const State& goal, double alpha)
{
    // Clamp alpha to [0, 1] just in case
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    // alpha = std::clamp(alpha, 0.0, 1.0); 
    // Another option for clamping

    State out{};
    
    out.theta1 = start.theta1 + alpha * (goal.theta1 - start.theta1);
    out.theta2 = start.theta2 + alpha * (goal.theta2 - start.theta2);

    const double dtheta1{goal.theta1 - start.theta1};
    const double dtheta2{goal.theta2 - start.theta2};

    // Assigning velocities proportional to dtheta
    out.dtheta1 = k_vel_limit * dtheta1;   
    out.dtheta2 = k_vel_limit * dtheta2;
    return out;
}

/**
 * @brief Declaration of a joint velocity constraint filter
 * 
 * @param traj 
 * @param filter 
 */
void apply_filter(std::vector<JointState>& traj,
                  std::function<JointState(const JointState&)> filter);
