/**
 * @file robot_control.cpp
 * @author Pranav Jagdish Koli
 * @brief The robot control cpp file consisting of a velocity limit filter.
 * @version 0.1
 * @date 2025-11-5
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "robot_control.hpp"


/**
 * @brief Definition of a simple joint velocity constraint filter to each element
 */
void apply_filter(std::vector<JointState> &traj,
                  std::function<JointState(const JointState &)> filter)
{
    int count_clamped_states{0};
    for (auto &s : traj)
    {
        s = filter(s);

        //Extending this to report how many states were clamped.
        if (std::abs(s.dtheta1) == k_vel_limit || std::abs(s.dtheta2 == k_vel_limit))
        {
            count_clamped_states++;
        }
        
    }
    std::cout << "Total states clamped = " << count_clamped_states << '\n';
}
