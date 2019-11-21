/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

namespace trackjoint
{
/**
 * \brief Maximum dynamics of Cartesian pose, parameters of TrackPose
 */
struct Limits
{
  // Scalar linear velocity limit [m/s]
  double linear_velocity_limit;
  // Scalar linear acceleration limit [m/s^2]
  double linear_acceleration_limit;
  // Scalar linear jerk limit [m/s^3]
  double linear_jerk_limit;

  // Tolerance on successful velocity matching [m/s]
  double linear_velocity_tolerance;
  // Tolerance on successful acceleration matching [m/s^s]
  double linear_acceleration_tolerance;
  // Tolerance on successful jerk matching [m/s^3]
  double linear_jerk_tolerance;

  // Scalar limit on angular velocity [rad/s]
  double angular_velocity_limit;
  // Scalar limit on angular acceleration [rad/s^2]
  double angular_acceleration_limit;
  // Scalar limit on angular jerk [rad/s^3]
  double angular_jerk_limit;

  // Tolerance on successful velocity matching [rad/s]
  double angular_velocity_tolerance;
  // Tolerance on successful acceleration matching [rad/s^2]
  double angular_acceleration_tolerance;
  // Tolerance on successful jerk matching [rad/s^3]
  double angular_jerk_tolerance;
};

}  // namespace trackjoint
