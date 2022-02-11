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
   Desc: Hold kinematics data for a robot joint trajectory.
*/

#pragma once

#include <Eigen/Dense>

namespace trackjoint
{
constexpr int EIGEN_AUTO_ALIGN = 0;
// Custom Eigen types using long double for better accuracy
typedef Eigen::Matrix<long double, Eigen::Dynamic, 1, EIGEN_AUTO_ALIGN> VectorXlong;
typedef Eigen::Matrix<long double, 1, Eigen::Dynamic, EIGEN_AUTO_ALIGN> RowVectorXlong;

/**
 * \brief Vectors of kinematic states and time
 */
struct JointTrajectory
{
  VectorXlong positions;      // rad
  VectorXlong velocities;     // rad/s
  VectorXlong accelerations;  // rad/s^2
  VectorXlong jerks;          // rad/s^3
  VectorXlong elapsed_times;  // seconds
};
}  // namespace trackjoint
