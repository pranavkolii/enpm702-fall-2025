/**
 * @file robot_control.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-10-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "robot_control.hpp"

/*
TODO: Simple in-place filter application loop (Task 3)
TODO: Remove this block of comment before submission

*/
/**
 * @brief A simple in-place filter to each element
 */
void apply_filter(std::vector<JointState> &traj,
                  std::function<JointState(const JointState &)> filter)
{
    // TODO [Task 3]: You may extend this to report how many states were clamped, etc.
    int count_clamped_states{0};
    for (auto &s : traj)
    {
        s = filter(s);
        if (std::abs(s.dtheta1) == k_vel_limit || std::abs(s.dtheta2 == k_vel_limit))
        {
            count_clamped_states++;
        }
        
    }
    std::cout << "Total states clamped = " << count_clamped_states << '\n';
}
