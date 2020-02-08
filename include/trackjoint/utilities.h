/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: Generate jerk-limited trajectories for a single joint.
*/

#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "trackjoint/joint_trajectory.h"

namespace trackjoint
{
/**
 * \brief Discrete differentiation of a vector
 *
 * input: first_element: we usually have an initial condition, so supply it
 * directly.
 */
Eigen::VectorXd DiscreteDifferentiation(const Eigen::VectorXd& input_vector, double timestep, double first_element);

void PrintJointTrajectory(const std::size_t joint, const std::vector<trackjoint::JointTrajectory>& output_trajectories,
                          const double desired_duration);
}  // namespace trackjoint
