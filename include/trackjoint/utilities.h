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
// Spline-fitting is used to extend trajectory duration and in derivative calculation
#include <unsupported/Eigen/Splines>
#include <vector>
#include "trackjoint/joint_trajectory.h"

namespace trackjoint
{
// Default to use spline of dynamic degree
typedef Eigen::Spline<double, 1 /* dimension */> Spline1D;
typedef Eigen::SplineFitting<Spline1D> SplineFitting1D;

/**
 * \brief Discrete differentiation of a vector. This is faster but noisier than the spline version.
 *
 * input input_vector any vector, such as position
 * input timestep the time between consecutive elements
 * input first_element supply an initial condition
 * return a vector of derivatives
 */
Eigen::VectorXd discreteDifferentiation(const Eigen::VectorXd& input_vector, const double timestep,
                                        const double first_element);

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
Eigen::VectorXd discreteDifferentiationWithFiltering(const Eigen::VectorXd& input_vector, const double timestep,
                                                     const double first_element, const double filter_coefficient);

/**
 * \brief Normalize a vector between 0 and 1
 *
 * input input_vector any vector, such as position
 * return a vector of normalized values between 0 and 1
 */
Eigen::VectorXd normalize(const Eigen::VectorXd& x);

/**
 * \brief Interpolate with splines then take the derivative.
 *
 * input input_vector any vector, such as position
 * input timestep the time between consecutive elements
 * input first_element supply an initial condition
 * return a vector of derivatives
 */
Eigen::VectorXd splineDifferentiation(const Eigen::VectorXd& input_vector, double timestep, double first_element);

/** \brief Print desired duration, number of waypoints, timestep, initial state, and final state of a trajectory
 *
 * input joint the index of a joint
 * input output_trajectories the calculated trajectories for n joints
 * input desired_duration the user-requested duration of the trajectory
 */
void printJointTrajectory(const std::size_t joint, const std::vector<JointTrajectory>& output_trajectories,
                          const double desired_duration);

/** \brief Clip all elements beyond a given size */
void clipEigenVector(Eigen::VectorXd* vector, size_t new_num_waypoints);

/* \brief Check if all elements of a vector are within a [low_limit, high_limit] range. */
bool verifyVectorWithinBounds(double low_limit, double high_limit, Eigen::VectorXd& vector);
}  // namespace trackjoint
