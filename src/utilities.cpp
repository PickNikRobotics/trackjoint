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
VectorXlong normalize(const VectorXlong& x)
{
  const double min = x.minCoeff();
  const double max = x.maxCoeff();
  size_t num_points = x.size();
  Eigen::Matrix<long double, 1, Eigen::Dynamic> x_norm = Eigen::Matrix<long double, 1, Eigen::Dynamic>::Zero(num_points);

  x_norm = (x.array() - min) / (max - min);
  return x_norm;
}

// TODO(andyz): This is horribly inefficient. Maybe it would be best to store everything as splines always.
// Also (maybe) to store everything in one large matrix with columns for each derivative.
VectorXlong splineDifferentiation(const VectorXlong& input_vector, double timestep, double first_element)
{
  // Fit a spline through the input vector
  size_t num_points = input_vector.size();
  Eigen::Matrix<long double, 1, Eigen::Dynamic> times = Eigen::Matrix<long double, 1, Eigen::Dynamic>::Zero(1, num_points);
  Eigen::Matrix<long double, 1, Eigen::Dynamic> input_row = Eigen::Matrix<long double, 1, Eigen::Dynamic>::Zero(1, num_points);
  for (size_t i = 1; i < num_points; ++i)
  {
    input_row(i) = input_vector(i);
    times(i) = times(i - 1) + timestep;
  }

  const auto knots = normalize(times);
  const auto fit = SplineFitting1D::Interpolate(input_row, 3, knots);
  double scale = 1 / (times.maxCoeff() - times.minCoeff());

  // Derivative output
  Eigen::Matrix<long double, 1, Eigen::Dynamic> derivative = Eigen::Matrix<long double, 1, Eigen::Dynamic>::Zero(1, num_points);
  derivative(0) = first_element;

  // Take derivatives
  double path_fraction = 0.0;
  for (size_t point = 1; point < num_points; ++point)
  {
    path_fraction = (double)point / (double)num_points;
    derivative[point] = fit.derivatives(path_fraction, 1 /* order */)(1 /* first derivative */) * scale;
  }

  return derivative.transpose();
}

void clipEigenVector(VectorXlong* vector, size_t new_num_waypoints)
{
  VectorXlong new_vector = vector->head(new_num_waypoints);
  *vector = new_vector;
}

bool verifyVectorWithinBounds(double low_limit, double high_limit, VectorXlong& vector)
{
  if (high_limit < low_limit)
    return false;

  return ((vector.array() >= low_limit).all() && (vector.array() <= high_limit).all());
}
}  // namespace trackjoint
