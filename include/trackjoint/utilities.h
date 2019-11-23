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

namespace trackjoint {
/**
 * \brief Discrete differentiation of a vector
 */
Eigen::VectorXd DiscreteDifferentiation(const Eigen::VectorXd &input_vector, double timestep)
{
  // derivative = (difference between adjacent elements) / timestep
  Eigen::VectorXd input_shifted_right(input_vector.size());
  input_shifted_right(0) = 0;
  input_shifted_right.tail(input_shifted_right.size() - 1) = 
    input_vector.head(input_vector.size()-1);
  Eigen::VectorXd derivative = (input_vector - input_shifted_right) / timestep;
  return derivative;
};
}  // namespace trackjoint
