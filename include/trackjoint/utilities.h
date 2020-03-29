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
inline Eigen::VectorXd DiscreteDifferentiation(const Eigen::VectorXd& input_vector, double timestep,
                                               double first_element)
{
  // derivative = (difference between adjacent elements) / timestep
  Eigen::VectorXd input_shifted_right(input_vector.size());
  input_shifted_right(0) = 0;
  input_shifted_right.tail(input_shifted_right.size() - 1) = input_vector.head(input_vector.size() - 1);
  Eigen::VectorXd derivative(input_vector.size());
  derivative(0) = first_element;
  derivative.tail(derivative.size() - 1) =
      (input_vector.tail(input_vector.size() - 1) - input_shifted_right.tail(input_shifted_right.size() - 1)) /
      timestep;

  return derivative;
};

/** \brief Print desired duration, number of waypoints, timestep, initial state, and final state of a trajectory
 *
 * input joint the index of a joint
 * input output_trajectories the calculated trajectories for n joints
 * input desired_duration the user-requested duration of the trajectory
*/
void PrintJointTrajectory(const std::size_t joint, const std::vector<trackjoint::JointTrajectory>& output_trajectories,
                          const double desired_duration);

/** \brief Clip all elements beyond a given size
 *
 * input vector a vector to clip, such as positions
 * input new_num_waypoints clip all elements beyond this number
*/
inline void ClipEigenVector(Eigen::VectorXd* vector, size_t new_num_waypoints)
{
  Eigen::VectorXd new_vector = vector->head(new_num_waypoints);
  *vector = new_vector;
}
}  // namespace trackjoint
