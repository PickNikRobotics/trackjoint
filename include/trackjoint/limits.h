/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

#pragma once

namespace trackjoint
{
/**
 * \brief Maximum dynamics of Cartesian pose, parameters of TrackPose
 */
struct Limits
{
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
