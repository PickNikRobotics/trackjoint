/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: Simple utilities, such as printing important data from a trajectory.
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
 * input input_vector any vector, such as position
 * input timestep the time between consecutive elements
 * input first_element supply an initial condition
 * return a vector of derivatives
 */
// TODO(602): Overload DiscreteDifferentiation to take starting index
Eigen::VectorXd DiscreteDifferentiation(const Eigen::VectorXd& input_vector, double timestep, double first_element);

/** \brief Print desired duration, number of waypoints, timestep, initial state, and final state of a trajectory
 *
 * input joint the index of a joint
 * input output_trajectories the calculated trajectories for n joints
 * input desired_duration the user-requested duration of the trajectory
 */
void PrintJointTrajectory(const std::size_t joint, const std::vector<JointTrajectory>& output_trajectories,
                          const double desired_duration);

/** \brief Clip all elements beyond a given size */
void ClipEigenVector(Eigen::VectorXd* vector, size_t new_num_waypoints);

/* \brief Check if all elements of a vector are within a [low_limit, high_limit] range. */
bool VerifyVectorWithinBounds(double low_limit, double high_limit, Eigen::VectorXd& vector);
}  // namespace trackjoint
