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
#include "trackjoint/butterworth_filter.h"
#include "trackjoint/joint_trajectory.h"

namespace trackjoint
{
// Default to use spline of dynamic degree
typedef Eigen::Spline<long double, 1 /* dimension */> Spline1D;
typedef Eigen::SplineFitting<Spline1D> SplineFitting1D;

/**
 * \brief Discrete differentiation of a vector. This is faster but noisier than the spline version.
 *
 * input input_vector any vector, such as position
 * input timestep the time between consecutive elements
 * input first_element supply an initial condition
 * return a vector of derivatives
 */
inline VectorXlong discreteDifferentiation(const VectorXlong& input_vector, double timestep, const double first_element)
{
  // derivative = (difference between adjacent elements) / timestep
  VectorXlong input_shifted_right(input_vector.size());
  input_shifted_right(0) = 0;
  input_shifted_right.tail(input_shifted_right.size() - 1) = input_vector.head(input_vector.size() - 1);
  VectorXlong derivative(input_vector.size());
  derivative(0) = first_element;
  derivative.tail(derivative.size() - 1) =
      (input_vector.tail(input_vector.size() - 1) - input_shifted_right.tail(input_shifted_right.size() - 1)) /
      timestep;

  return derivative;
}

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
inline VectorXlong discreteDifferentiationWithFiltering(const VectorXlong& input_vector, const double timestep,
                                                     const double first_element, const double filter_coefficient)
{
  VectorXlong derivative = discreteDifferentiation(input_vector, timestep, first_element);

  // Apply a low-pass filter
  ButterworthFilter filter(filter_coefficient);

  // Filter from front to back
  filter.reset(derivative(0));
  for (long int point = 1; point < derivative.size(); ++point)
  {
    // Lowpass filter the position command
    derivative(point) = filter.filter(derivative(point));
  }

  // Now filter from back to front to eliminate phase shift
  filter.reset(derivative(derivative.size() - 1));
  for (size_t point = derivative.size() - 2; point > 0; --point)
  {
    // Lowpass filter the position command
    derivative(point) = filter.filter(derivative(point));
  }

  return derivative;
}

/**
 * \brief Normalize a vector between 0 and 1
 *
 * input input_vector any vector, such as position
 * return a vector of normalized values between 0 and 1
 */
VectorXlong normalize(const VectorXlong& x);

/**
 * \brief Interpolate with splines then take the derivative.
 *
 * input input_vector any vector, such as position
 * input timestep the time between consecutive elements
 * input first_element supply an initial condition
 * return a vector of derivatives
 */
VectorXlong splineDifferentiation(const VectorXlong& input_vector, long double timestep, long double first_element);

/** \brief Print desired duration, number of waypoints, timestep, initial state, and final state of a trajectory
 *
 * input joint the index of a joint
 * input output_trajectories the calculated trajectories for n joints
 * input desired_duration the user-requested duration of the trajectory
 */
inline void printJointTrajectory(const std::size_t joint, const std::vector<JointTrajectory>& output_trajectories,
                          const double desired_duration)
{
  std::cout << "==========" << '\n';
  std::cout << '\n';
  std::cout << '\n';
  std::cout << "==========" << '\n';
  std::cout << "Joint " << joint << '\n';
  std::cout << "==========" << '\n';
  std::cout << "  Num waypts: " << output_trajectories.at(joint).positions.size() << '\n';
  std::cout << "  Desired duration: " << desired_duration << '\n';
  std::cout << "Timestep: " << output_trajectories.at(joint).elapsed_times[1]
            << "  Initial position: " << output_trajectories.at(joint).positions[0]
            << "  Initial velocity: " << output_trajectories.at(joint).velocities[0]
            << "  Initial acceleration: " << output_trajectories.at(joint).accelerations[0]
            << "  Initial jerk: " << output_trajectories.at(joint).jerks[0] << '\n';
  std::cout << "  Final position: "
            << output_trajectories.at(joint).positions[output_trajectories.at(joint).positions.size() - 1]
            << "  Final velocity: "
            << output_trajectories.at(joint).velocities[output_trajectories.at(joint).positions.size() - 1]
            << "  Final acceleration: "
            << output_trajectories.at(joint).accelerations[output_trajectories.at(joint).positions.size() - 1]
            << "  Final jerk: "
            << output_trajectories.at(joint).jerks[output_trajectories.at(joint).positions.size() - 1] << '\n';
}

/** \brief Clip all elements beyond a given size */
void clipEigenVector(VectorXlong* vector, size_t new_num_waypoints);

/* \brief Check if all elements of a vector are within a [low_limit, high_limit] range. */
bool verifyVectorWithinBounds(long double low_limit, long double high_limit, VectorXlong& vector);
}  // namespace trackjoint
