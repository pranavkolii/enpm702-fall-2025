/**
 * @file robot_control.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-10-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once
#include "robot_types.hpp"
#include <vector>
#include <functional>
 

/**
 * @brief 
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
    // TODO [Task 3]:
    //  - Interpolate theta1, theta2 linearly
    //  - Set dtheta1, dtheta2 proportional to (goal - start)
    //
    // Starter: angles interpolated, velocities placeholder (0)
    out.theta1 = start.theta1 + alpha * (goal.theta1 - start.theta1);
    out.theta2 = start.theta2 + alpha * (goal.theta2 - start.theta2);

    const double dtheta1{goal.theta1 - start.theta1};
    const double dtheta2{goal.theta2 - start.theta2};

    out.dtheta1 = k_vel_limit * dtheta1;
    out.dtheta2 = k_vel_limit * dtheta2;
    return out;
}

//---------------------------------------------------------
// TODO: apply_filter (Task 3, non-template)
// - declared here, implemented in .cpp
// TODO: Remove this block of comment before submission
//---------------------------------------------------------
/**
 * @brief 
 * 
 * @param traj 
 * @param filter 
 */
void apply_filter(std::vector<JointState>& traj,
                  std::function<JointState(const JointState&)> filter);
