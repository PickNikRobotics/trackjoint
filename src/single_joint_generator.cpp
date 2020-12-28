/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

#include "trackjoint/single_joint_generator.h"

namespace trackjoint
{
SingleJointGenerator::SingleJointGenerator(size_t num_waypoints_threshold, size_t max_num_waypoints_trajectory_mode)
  : kNumWaypointsThreshold(num_waypoints_threshold)
  , kMaxNumWaypointsFullTrajectory(max_num_waypoints_trajectory_mode)
  , successful_limit_comp_(false)
  , is_reset_(false)

{
}

void SingleJointGenerator::reset(const Configuration& configuration, const KinematicState& current_joint_state,
                                 const KinematicState& goal_joint_state, size_t desired_num_waypoints,
                                 bool timestep_was_upsampled)
{
  configuration_ = configuration;
  current_joint_state_ = current_joint_state;
  goal_joint_state_ = goal_joint_state;
  successful_limit_comp_ = false;
  is_reset_ = true;

  // Start with this estimate of the shortest possible duration
  // The shortest possible duration avoids oscillation, as much as possible
  // Desired duration cannot be less than one timestep
  if (!timestep_was_upsampled)
  {
    desired_duration_ =
        std::max(configuration.timestep, fabs((goal_joint_state.position - current_joint_state.position) /
                                              configuration_.limits.velocity_limit));
  }
  // If upsampling was used, we don't want to mess with the timestep or duration minimization
  else
  {
    desired_duration_ = (desired_num_waypoints - 1) * configuration.timestep;
  }

  // Waypoint times
  nominal_times_ = Eigen::VectorXd::LinSpaced(desired_num_waypoints, 0, desired_duration_);
}

void SingleJointGenerator::reset(double timestep, double max_duration, const KinematicState& current_joint_state,
                                 const KinematicState& goal_joint_state, const Limits& limits,
                                 size_t desired_num_waypoints, const double position_tolerance,
                                 bool timestep_was_upsampled)
{
  Configuration configuration(timestep, max_duration, limits, position_tolerance);
  this->reset(configuration, current_joint_state, goal_joint_state, desired_num_waypoints, timestep_was_upsampled);
}

ErrorCodeEnum SingleJointGenerator::generateTrajectory()
{
  if (!is_reset_)
    return ErrorCodeEnum::OBJECT_NOT_RESET;

  // Clear previous results
  waypoints_ = JointTrajectory();
  waypoints_.positions = interpolate(nominal_times_);

  waypoints_.elapsed_times.setLinSpaced(waypoints_.positions.size(), 0.,
                                        (waypoints_.positions.size() - 1) * configuration_.timestep);
  calculateDerivativesFromPosition();

  ErrorCodeEnum error_code = positionVectorLimitLookAhead(successful_limit_comp_);
  if (error_code)
  {
    return error_code;
  }

  // Extend the trajectory duration, if needed
  error_code = predictTimeToReach();

  return error_code;
}

void SingleJointGenerator::extendTrajectoryDuration()
{
  size_t new_num_waypoints = 1 + desired_duration_ / configuration_.timestep;

  // If waypoints were successfully generated for this dimension previously, just stretch the trajectory with splines.
  // ^This is the best way because it reduces overshoot.
  // Otherwise, re-generate a new trajectory from scratch.
  if (successful_limit_comp_)
  {
    // Fit and generate a spline function to the original positions, same number of waypoints, new (extended) duration
    // This only decreases velocity/accel/jerk, so no worries re. limit violation
    Eigen::RowVectorXd new_times;
    new_times.setLinSpaced(waypoints_.elapsed_times.size(), 0, desired_duration_);
    Eigen::RowVectorXd position(waypoints_.positions);

    const auto fit = SplineFitting1D::Interpolate(position, 2, new_times);

    // New times, with the extended duration
    waypoints_.elapsed_times.setLinSpaced(new_num_waypoints, 0., (new_num_waypoints - 1) * configuration_.timestep);
    // Retrieve new positions at the new times
    waypoints_.positions.resize(new_num_waypoints);

    for (Eigen::Index idx = 0; idx < waypoints_.elapsed_times.size(); ++idx)
      waypoints_.positions[idx] = fit(waypoints_.elapsed_times(idx)).coeff(0);

    calculateDerivativesFromPosition();
    return;
  }

  // Plan a new trajectory from scratch:
  // Clear previous results
  else
  {
    waypoints_ = JointTrajectory();
    waypoints_.elapsed_times.setLinSpaced(new_num_waypoints, 0., (new_num_waypoints - 1) * configuration_.timestep);
    waypoints_.positions = interpolate(waypoints_.elapsed_times);
    calculateDerivativesFromPosition();
    forwardLimitCompensation(successful_limit_comp_);
  }

  return;
}

JointTrajectory SingleJointGenerator::getTrajectory()
{
  return waypoints_;
}

size_t SingleJointGenerator::getTrajectoryLength()
{
  return waypoints_.positions.size();
}

Eigen::VectorXd SingleJointGenerator::interpolate(Eigen::VectorXd& times)
{
  // See De Luca, "Trajectory Planning" pdf, slide 19
  // Interpolate a smooth trajectory from initial to final state while matching
  // boundary conditions.

  // De Luca uses tao to represent a normalized time
  Eigen::VectorXd tao = times / desired_duration_;

  // TODO(andyz): vectorize this calculation
  Eigen::VectorXd interpolated_position(tao.size());
  for (size_t index = 0; index < static_cast<size_t>(tao.size()); ++index)
  {
    interpolated_position(index) =
        pow((1. - tao(index)), 3) *
            (current_joint_state_.position +
             (3. * current_joint_state_.position + current_joint_state_.velocity * desired_duration_) * tao(index) +
             (current_joint_state_.acceleration * pow(desired_duration_, 2) +
              6. * current_joint_state_.velocity * desired_duration_ + 12. * current_joint_state_.position) *
                 pow(tao(index), 2) / 2.) +
        pow(tao(index), 3) *
            (goal_joint_state_.position +
             (3. * goal_joint_state_.position - goal_joint_state_.velocity * desired_duration_) * (1. - tao(index)) +
             (goal_joint_state_.acceleration * pow(desired_duration_, 2) -
              6. * goal_joint_state_.velocity * desired_duration_ + 12. * goal_joint_state_.position) *
                 pow((1. - tao(index)), 2) / 2.);
  }

  return interpolated_position;
}

ErrorCodeEnum SingleJointGenerator::forwardLimitCompensation(bool& successful_limit_comp)
{
  // This is the indexing convention.
  // 1. accel(i) = accel(i-1) + jerk(i) * dt
  // 2. vel(i) == vel(i-1) + accel(i-1) * dt + 0.5 * jerk(i) * dt ^ 2

  successful_limit_comp = false;

  // Discrete differentiation introduces small numerical errors, so allow a small tolerance
  const double limit_relative_tol = 0.999999;
  const double jerk_limit = limit_relative_tol * configuration_.limits.jerk_limit;
  const double acceleration_limit = limit_relative_tol * configuration_.limits.acceleration_limit;
  const double velocity_limit = limit_relative_tol * configuration_.limits.velocity_limit;

  // Preallocate
  double delta_a(0), delta_v(0), position_error(0);

  // Compensate for jerk limits at each timestep, starting near the beginning
  // Do not want to affect vel/accel at the first/last timestep
  bool successful_jerk_comp = true;
  for (int index = 1; index < waypoints_.positions.size() - 1; ++index)
  {
    if (fabs((waypoints_.accelerations(index) - waypoints_.accelerations(index - 1)) / configuration_.timestep) > jerk_limit)
    {
      double delta_j = std::copysign(jerk_limit, waypoints_.jerks(index)) - waypoints_.jerks(index);
      waypoints_.jerks(index) = std::copysign(jerk_limit, waypoints_.jerks(index));

      delta_a = delta_j * configuration_.timestep;
      waypoints_.accelerations(index) =
          waypoints_.accelerations(index - 1) + waypoints_.jerks(index) * configuration_.timestep;
      waypoints_.velocities(index) = waypoints_.velocities(index - 1) +
                                     waypoints_.accelerations(index - 1) * configuration_.timestep +
                                     0.5 * waypoints_.jerks(index) * configuration_.timestep * configuration_.timestep;

      delta_v = position_error / configuration_.timestep + delta_a * configuration_.timestep + 0.5 * delta_j * configuration_.timestep * configuration_.timestep;

      // Try adjusting the velocity in previous timesteps to compensate for this limit, if needed
      delta_v += position_error / configuration_.timestep;
      successful_jerk_comp = backwardLimitCompensation(index, -delta_v);
      if (!successful_jerk_comp)
      {
        position_error = position_error + delta_v * configuration_.timestep;
      }
      else
        position_error = 0;
    }
  }

  // Compensate for acceleration limits at each timestep, starting near the beginning of the trajectory.
  // Do not want to affect user-provided acceleration at the first timestep, so start at index 2.
  // Also do not want to affect user-provided acceleration at the last timestep.
  bool successful_acceleration_comp = true;
  for (int index = 1; index < waypoints_.positions.size() - 1; ++index)
  {
    if (fabs((waypoints_.velocities(index) - waypoints_.velocities(index - 1)) / configuration_.timestep) > acceleration_limit)
    {
      double temp_accel = std::copysign(acceleration_limit, waypoints_.accelerations(index));
      delta_a = temp_accel - waypoints_.accelerations(index);

      // Check jerk limit before applying the change.
      // The first condition checks if the new jerk(i) is going to exceed the limit. Pretty straightforward.
      // We also calculate a new jerk(i+1). The second condition checks if jerk(i+1) would exceed the limit.
      if ((fabs((temp_accel - waypoints_.accelerations(index - 1)) / configuration_.timestep) <= jerk_limit) &&
          (fabs((waypoints_.accelerations(index + 1) - temp_accel) / configuration_.timestep) <= jerk_limit))
      {
        waypoints_.accelerations(index) = temp_accel;
        waypoints_.jerks(index) =
            (waypoints_.accelerations(index) - waypoints_.accelerations(index - 1)) / configuration_.timestep;
        waypoints_.velocities(index) =
            waypoints_.velocities(index - 1) + waypoints_.accelerations(index - 1) * configuration_.timestep +
            0.5 * waypoints_.jerks(index) * configuration_.timestep * configuration_.timestep;

        // Try adjusting the velocity in previous timesteps to compensate for this limit, if needed
        // Try to account for position error, too.
        delta_v = delta_a * configuration_.timestep;
        successful_acceleration_comp = backwardLimitCompensation(index, -delta_v);
      }
      else
      {
        successful_limit_comp = false;
        return ErrorCodeEnum::NO_ERROR;
      }
    }
  }

  // Compensate for velocity limits at each timestep, starting near the beginning of the trajectory.
  // Do not want to affect user-provided velocity at the first timestep, so start at index 2.
  // Also do not want to affect user-provided velocity at the last timestep.
  double successful_velocity_comp = true;
  for (int index = 1; index < waypoints_.positions.size() - 1; ++index)
  {
    // If the velocity limit would be exceeded
    if (fabs(waypoints_.velocities(index)) > velocity_limit)
    {
      delta_v = std::copysign(velocity_limit, waypoints_.velocities(index)) - waypoints_.velocities(index);
      waypoints_.velocities(index) = std::copysign(velocity_limit, waypoints_.velocities(index));

      // Try adjusting the velocity in previous timesteps to compensate for this limit.
      // Try to account for position error, too.
      delta_v += position_error / configuration_.timestep;
      successful_velocity_comp = backwardLimitCompensation(index, -delta_v);
      if (!successful_velocity_comp)
      {
        position_error = position_error + delta_v * configuration_.timestep;
      }
      else
      {
        position_error = 0;
      }
    }
  }

  // Re-calculate derivatives from the updated velocity vector
  calculateDerivativesFromVelocity();

  // Check for success
  // TODO(andyz): check limits too
  if (successful_jerk_comp && successful_acceleration_comp && successful_velocity_comp &&
      (position_error < configuration_.position_tolerance))
    successful_limit_comp = true;
  else
    successful_limit_comp = false;

  return ErrorCodeEnum::NO_ERROR;
}

bool SingleJointGenerator::backwardLimitCompensation(size_t limited_index, double excess_velocity)
{
  // The algorithm:
  // 1) check jerk limits, from beginning to end of trajectory. Don't bother
  // checking accel/vel limits here, they will be checked next
  // 2) check accel limits. Make sure it doesn't cause jerk to exceed limits.
  // 3) check vel limits. This will also check whether previously-checked
  // jerk/accel limits were exceeded

  bool successful_compensation = false;

  // Add a bit of velocity at step i to compensate for the limit at timestep i+1.
  // Cannot go beyond index 2 because we use a 2-index window for derivative calculations.
  for (size_t index = limited_index; index > 2; --index)
  {
    // if there is some room to increase the velocity at timestep i
    if (fabs(waypoints_.velocities(index)) < configuration_.limits.velocity_limit)
    {
      // If the full change can be made in this timestep
      if ((excess_velocity > 0 &&
           waypoints_.velocities(index) <= configuration_.limits.velocity_limit - excess_velocity) ||
          (excess_velocity < 0 &&
           waypoints_.velocities(index) >= -configuration_.limits.velocity_limit - excess_velocity))
      {
        double new_velocity = waypoints_.velocities(index) + excess_velocity;
        // Accel and jerk, calculated from the previous waypoints
        double backward_accel = (new_velocity - waypoints_.velocities(index - 1)) / configuration_.timestep;
        double backward_jerk = (backward_accel - (waypoints_.velocities(index - 1) - waypoints_.velocities(index - 2)) /
                                                     configuration_.timestep) /
                               configuration_.timestep;
        // Accel and jerk, calculated from upcoming waypoints
        double forward_accel = (waypoints_.velocities(index + 1) - new_velocity) / configuration_.timestep;
        double forward_jerk =
            (forward_accel - (new_velocity - waypoints_.velocities(index - 1)) / configuration_.timestep) /
            configuration_.timestep;

        // Calculate this new velocity if it would not violate accel/jerk limits
        if ((fabs(backward_jerk) < configuration_.limits.jerk_limit) &&
            (fabs(backward_accel) < configuration_.limits.acceleration_limit) &&
            (fabs(forward_jerk) < configuration_.limits.jerk_limit) &&
            (fabs(forward_accel) < configuration_.limits.acceleration_limit))
        {
          waypoints_.velocities(index) = new_velocity;

          // if at the velocity limit, must stop accelerating
          if (fabs(waypoints_.velocities(index)) >= configuration_.limits.velocity_limit)
          {
            waypoints_.accelerations(index) = 0;
            waypoints_.jerks(index) = 0;
          }
          else
          {
            waypoints_.accelerations(index) = backward_accel;
            waypoints_.jerks(index) = backward_jerk;
          }

          successful_compensation = true;
          break;
        }
      }
      // Can't make all of the correction in this timestep, so make as much of a change as possible
      if (!successful_compensation)
      {
        // This is what accel and jerk would be if we set velocity(index) to the limit
        double new_velocity = std::copysign(1.0, excess_velocity) * configuration_.limits.velocity_limit;
        // Accel and jerk, calculated from the previous waypoints
        double backward_accel = (new_velocity - waypoints_.velocities(index - 1)) / configuration_.timestep;
        double backward_jerk = (backward_accel - (waypoints_.velocities(index - 1) - waypoints_.velocities(index - 2)) /
                                                     configuration_.timestep) /
                               configuration_.timestep;
        // Accel and jerk, calculated from upcoming waypoints
        double forward_accel = (waypoints_.velocities(index + 1) - new_velocity) / configuration_.timestep;
        double forward_jerk =
            (forward_accel - (new_velocity - waypoints_.velocities(index - 1)) / configuration_.timestep) /
            configuration_.timestep;

        // Apply these changes if all limits are satisfied (for the prior
        // waypoint and the future waypoint)
        if ((fabs(backward_accel) < configuration_.limits.acceleration_limit) &&
            (fabs(backward_jerk) < configuration_.limits.jerk_limit) &&
            (fabs(forward_accel) < configuration_.limits.acceleration_limit) &&
            (fabs(forward_jerk) < configuration_.limits.jerk_limit))
        {
          double delta_v = new_velocity - waypoints_.velocities(index);
          waypoints_.velocities(index) = new_velocity;

          // if at the velocity limit, must stop accelerating
          if (fabs(waypoints_.velocities(index)) >= configuration_.limits.velocity_limit)
          {
            waypoints_.accelerations(index) = 0;
            waypoints_.jerks(index) = 0;
          }
          else
          {
            waypoints_.accelerations(index) = backward_accel;
            waypoints_.jerks(index) = backward_jerk;
          }
          excess_velocity -= delta_v;
        }
      }
    }
    // Clip velocities at min/max due to small rounding errors
    waypoints_.velocities(index) =
        std::min(std::max(waypoints_.velocities(index), -configuration_.limits.velocity_limit),
                 configuration_.limits.velocity_limit);
  }

  return successful_compensation;
}

ErrorCodeEnum SingleJointGenerator::predictTimeToReach()
{
  // Take a trajectory that could not reach the desired position in time.
  // Try increasing the duration until it is interpolated without violating limits.
  // This gives a new duration estimate.

  ErrorCodeEnum error_code = ErrorCodeEnum::NO_ERROR;

  size_t new_num_waypoints = 0;
  const double duration_extension_factor = 1.05;
  // Iterate over new durations until the position error is acceptable or the maximum duration is reached
  while (!successful_limit_comp_ &&
         ((duration_extension_factor * desired_duration_) < configuration_.max_duration) && (new_num_waypoints < kMaxNumWaypointsFullTrajectory))
  {
    // Try increasing the duration
    desired_duration_ = duration_extension_factor * desired_duration_;

    // // Round to nearest timestep
    if (std::fmod(desired_duration_, configuration_.timestep) > 0.5 * configuration_.timestep)
      desired_duration_ = desired_duration_ + configuration_.timestep;

    new_num_waypoints = std::max(static_cast<size_t>(waypoints_.positions.size() + 1),
                                 static_cast<size_t>(floor(1 + desired_duration_ / configuration_.timestep)));
    // Cap the trajectory duration to maintain determinism
    if (new_num_waypoints > kMaxNumWaypointsFullTrajectory)
      new_num_waypoints = kMaxNumWaypointsFullTrajectory;

    waypoints_.elapsed_times.setLinSpaced(new_num_waypoints, 0., (new_num_waypoints - 1) * configuration_.timestep);
    waypoints_.positions.resize(waypoints_.elapsed_times.size());
    waypoints_.velocities.resize(waypoints_.elapsed_times.size());
    waypoints_.accelerations.resize(waypoints_.elapsed_times.size());
    waypoints_.jerks.resize(waypoints_.elapsed_times.size());

    ////////////////////////////////////////////////////////////
    // Try to create the trajectory again, with the new duration
    ////////////////////////////////////////////////////////////
    waypoints_.positions = interpolate(waypoints_.elapsed_times);
    calculateDerivativesFromPosition();
    positionVectorLimitLookAhead(successful_limit_comp_);
  }

  if (!successful_limit_comp_)
  {
    error_code = ErrorCodeEnum::MAX_DURATION_EXCEEDED;
  }
  // Error if not even a single waypoint could be generated
  if (waypoints_.positions.size() < 2)
  {
    error_code = ErrorCodeEnum::FAILURE_TO_GENERATE_SINGLE_WAYPOINT;
  }

  return error_code;
}

ErrorCodeEnum SingleJointGenerator::positionVectorLimitLookAhead(bool& successful_limit_comp)
{
  ErrorCodeEnum error_code = forwardLimitCompensation(successful_limit_comp);
  if (error_code)
    return error_code;

  // Re-compile the position with these modifications.
  // Ensure the first and last elements are a perfect match with initial/final
  // conditions
  const double one_sixth = 0.166667;
  // Initial waypoint
  waypoints_.positions(0) = current_joint_state_.position;
  for (size_t index = 1; index < static_cast<size_t>(waypoints_.positions.size()); ++index)
    waypoints_.positions(index) = waypoints_.positions(index - 1) +
                                  waypoints_.velocities(index - 1) * configuration_.timestep +
                                  0.5 * waypoints_.accelerations(index - 1) * pow(configuration_.timestep, 2) +
                                  one_sixth * waypoints_.jerks(index - 1) * pow(configuration_.timestep, 3);

  return error_code;
}

void SingleJointGenerator::calculateDerivativesFromPosition()
{
  // From position vector, approximate vel/accel/jerk.
  waypoints_.velocities =
      DiscreteDifferentiation(waypoints_.positions, configuration_.timestep, current_joint_state_.velocity);
  waypoints_.accelerations =
      DiscreteDifferentiation(waypoints_.velocities, configuration_.timestep, current_joint_state_.acceleration);
  waypoints_.jerks = DiscreteDifferentiation(waypoints_.accelerations, configuration_.timestep, 0);
}

void SingleJointGenerator::calculateDerivativesFromVelocity()
{
  // From velocity vector, approximate accel/jerk.
  waypoints_.accelerations =
      DiscreteDifferentiation(waypoints_.velocities, configuration_.timestep, current_joint_state_.acceleration);
  waypoints_.jerks = DiscreteDifferentiation(waypoints_.accelerations, configuration_.timestep, 0);
}

void SingleJointGenerator::updateTrajectoryDuration(double new_trajectory_duration)
{
  // The trajectory will be forced to have this duration (or fail) because
  // max_duration == desired_duration
  desired_duration_ = new_trajectory_duration;
  configuration_.max_duration = new_trajectory_duration;
}
}  // end namespace trackjoint
