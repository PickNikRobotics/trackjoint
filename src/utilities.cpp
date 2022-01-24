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

#include "trackjoint/utilities.h"

namespace trackjoint
{
Eigen::VectorXd DiscreteDifferentiation(const Eigen::VectorXd& input_vector, double timestep, double first_element)
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

// TODO(andyz): This is horribly inefficient. Maybe it would be best to store everything as splines always.
// Also (maybe) to store everything in one large matrix with columns for each derivative.
Eigen::VectorXd SplineDifferentiation(const Eigen::VectorXd& input_vector, double timestep, double first_element)
{
  // Fit a spline through the input vector
  size_t num_points = input_vector.size();
  Eigen::RowVectorXd times = Eigen::RowVectorXd::Zero(num_points);
  Eigen::RowVectorXd input_row = Eigen::RowVectorXd::Zero(num_points);
  for (size_t i = 1; i < num_points; ++i)
  {
    input_row(i) = input_vector(i);
    times(i) = times(i-1) + timestep;
  }

  const auto fit = SplineFitting1D::Interpolate(input_row, 3, times);

  // Derivative output
  Eigen::RowVectorXd derivative = Eigen::RowVectorXd::Zero(num_points);
  derivative(0) = first_element;

  // Take derivatives
  double path_fraction = 0.0;
  for (size_t point = 1; point < num_points; ++point)
  {
    path_fraction = (double)point / (double)num_points;
    derivative[point] =
      fit.derivatives(path_fraction, 1 /* order */)(1 /* first derivative */);
  }

  return derivative.transpose();
};

void PrintJointTrajectory(const std::size_t joint, const std::vector<JointTrajectory>& output_trajectories,
                          const double desired_duration)
{
  std::cout << "==========" << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "==========" << std::endl;
  std::cout << "Joint " << joint << std::endl;
  std::cout << "==========" << std::endl;
  std::cout << "  Num waypts: " << output_trajectories.at(joint).positions.size() << std::endl;
  std::cout << "  Desired duration: " << desired_duration << std::endl;
  std::cout << "Timestep: " << output_trajectories.at(joint).elapsed_times[1]
            << "  Initial position: " << output_trajectories.at(joint).positions[0]
            << "  Initial velocity: " << output_trajectories.at(joint).velocities[0]
            << "  Initial acceleration: " << output_trajectories.at(joint).accelerations[0]
            << "  Initial jerk: " << output_trajectories.at(joint).jerks[0] << std::endl;
  std::cout << "  Final position: "
            << output_trajectories.at(joint).positions[output_trajectories.at(joint).positions.size() - 1]
            << "  Final velocity: "
            << output_trajectories.at(joint).velocities[output_trajectories.at(joint).positions.size() - 1]
            << "  Final acceleration: "
            << output_trajectories.at(joint).accelerations[output_trajectories.at(joint).positions.size() - 1]
            << "  Final jerk: "
            << output_trajectories.at(joint).jerks[output_trajectories.at(joint).positions.size() - 1] << std::endl;
}

void ClipEigenVector(Eigen::VectorXd* vector, size_t new_num_waypoints)
{
  Eigen::VectorXd new_vector = vector->head(new_num_waypoints);
  *vector = new_vector;
}

bool VerifyVectorWithinBounds(double low_limit, double high_limit, Eigen::VectorXd& vector)
{
  if (high_limit < low_limit)
    return false;

  return ((vector.array() >= low_limit).all() && (vector.array() <= high_limit).all());
}
}  // namespace trackjoint
