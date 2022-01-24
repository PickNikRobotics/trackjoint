// Copyright 2021 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
 * \brief Discrete differentiation of a vector. Fast but noisy.
 *
 * input input_vector any vector, such as position
 * input timestep the time between consecutive elements
 * input first_element supply an initial condition
 * return a vector of derivatives
 */
Eigen::VectorXd DiscreteDifferentiation(const Eigen::VectorXd& input_vector,
                                        const double timestep, double first_element);

/**
 * \brief Discrete differentiation of a vector followed by low-pass filtering.
 * This reduces signal noise.
 *
 * input input_vector any vector, such as position
 * input timestep the time between consecutive elements
 * input first_element supply an initial condition
 * input filter_coefficient must be >1.0, typically less than 100. Larger value -> more smoothing.
 * return a vector of derivatives
 */
Eigen::VectorXd DiscreteDifferentiationWithFiltering(const Eigen::VectorXd& input_vector,
                                                     const double timestep,
                                                     const double first_element,
                                                     const double filter_coefficient);

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
