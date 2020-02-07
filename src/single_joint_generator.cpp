/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

#include <trackjoint/single_joint_generator.h>
#include <trackjoint/utilities.h>

namespace trackjoint
{
SingleJointGenerator::SingleJointGenerator(double timestep, double desired_duration, double max_duration,
                                           const trackjoint::KinematicState& current_joint_state,
                                           const trackjoint::KinematicState& goal_joint_state,
                                           const trackjoint::Limits& limits, size_t desired_num_waypoints,
                                           size_t max_num_waypoints, const double position_tolerance)
  : kTimestep(timestep)
  , kCurrentJointState(current_joint_state)
  , kGoalJointState(goal_joint_state)
  , kLimits(limits)
  , kMaxNumWaypoints(max_num_waypoints)
  , kPositionTolerance(position_tolerance)
  , desired_duration_(desired_duration)
  , max_duration_(max_duration)
{
  // Waypoint times
  nominal_times_ = Eigen::VectorXd::LinSpaced(desired_num_waypoints, 0, desired_duration_);
}

ErrorCodeEnum SingleJointGenerator::GenerateTrajectory()
{
  // Clear previous results
  waypoints_ = JointTrajectory();
  waypoints_.positions = Interpolate(nominal_times_);
  waypoints_.elapsed_times.setLinSpaced(waypoints_.positions.size(), 0., desired_duration_);
  CalculateDerivatives();

  ErrorCodeEnum error_code = PositionVectorLimitLookAhead(&index_last_successful_);
  if (error_code)
  {
    return error_code;
  }

  error_code = PredictTimeToReach();

  return error_code;
}

ErrorCodeEnum SingleJointGenerator::ExtendTrajectoryDuration()
{
  // Clear previous results
  waypoints_ = JointTrajectory();
  size_t expected_num_waypoints = 1 + desired_duration_ / kTimestep;
  waypoints_.elapsed_times.setLinSpaced(expected_num_waypoints, 0., desired_duration_);
  waypoints_.positions = Interpolate(waypoints_.elapsed_times);
  CalculateDerivatives();
  ErrorCodeEnum error_code = LimitCompensation(&index_last_successful_);
  return error_code;
}

JointTrajectory SingleJointGenerator::GetTrajectory()
{
  return waypoints_;
}

size_t SingleJointGenerator::GetLastSuccessfulIndex()
{
  return index_last_successful_;
}

Eigen::VectorXd SingleJointGenerator::Interpolate(Eigen::VectorXd& times)
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
            (kCurrentJointState.position +
             (3. * kCurrentJointState.position + kCurrentJointState.velocity * desired_duration_) * tao(index) +
             (kCurrentJointState.acceleration * pow(desired_duration_, 2) +
              6. * kCurrentJointState.velocity * desired_duration_ + 12. * kCurrentJointState.position) *
                 pow(tao(index), 2) / 2.) +
        pow(tao(index), 3) *
            (kGoalJointState.position +
             (3. * kGoalJointState.position - kGoalJointState.velocity * desired_duration_) * (1. - tao(index)) +
             (kGoalJointState.acceleration * pow(desired_duration_, 2) -
              6. * kGoalJointState.velocity * desired_duration_ + 12. * kGoalJointState.position) *
                 pow((1. - tao(index)), 2) / 2.);
  }

  return interpolated_position;
}

ErrorCodeEnum SingleJointGenerator::LimitCompensation(size_t* index_last_successful)
{
  // This is the indexing convention:
  // accel(i) = accel(i-1) + jerk(i) * dt

  // Start with the assumption that the entire trajectory can be completed
  *index_last_successful = waypoints_.positions.size();
  bool successful_compensation = false;

  // Discrete differentiation introduces small numerical errors, so allow a small tolerance
  double kLimitRelativeTol = 0.999999;
  double jerk_limit = kLimitRelativeTol * kLimits.jerk_limit;
  double acceleration_limit = kLimitRelativeTol * kLimits.acceleration_limit;
  double velocity_limit = kLimitRelativeTol * kLimits.velocity_limit;

  // Preallocate
  double delta_j(0), delta_a(0), delta_v(0), position_error(0);
  size_t num_waypoints = waypoints_.positions.size();

  // Compensate for jerk limits at each timestep, starting near the beginning
  // Do not want to affect the velocity at the first/last timestep
  for (size_t index = 1; index < (num_waypoints - 1); ++index)
  {
    delta_j = 0;
    if (fabs(waypoints_.jerks(index)) > jerk_limit)
    {
      delta_j = std::copysign(1.0, waypoints_.jerks(index)) * jerk_limit - waypoints_.jerks(index);
      waypoints_.jerks(index) = std::copysign(1.0, waypoints_.jerks(index)) * jerk_limit;

      waypoints_.accelerations(index) = waypoints_.accelerations(index - 1) + waypoints_.jerks(index) * kTimestep;
      // Recalculate jerk(i+1) to keep accel(i+1) and vel(i+1) the same
      waypoints_.jerks(index + 1) = (waypoints_.accelerations(index + 1) - waypoints_.accelerations(index)) / kTimestep;

      waypoints_.velocities(index) = waypoints_.velocities(index) + waypoints_.accelerations(index) * kTimestep;
      delta_v = 0.5 * delta_j * kTimestep * kTimestep;

      // Try adjusting the velocity in previous timesteps to compensate for this
      // limit, if needed
      successful_compensation = VelocityCompensation(index, delta_v);
      if (!successful_compensation)
      {
        position_error = position_error + delta_v * kTimestep;
      }
      if (fabs(position_error) > kPositionTolerance)
      {
        RecordFailureTime(index, index_last_successful);
        // Only break, do not return, because we are looking for the FIRST failure. May find an earlier failure in
        // subsequent code
        break;
      }
    }
  }

  // Compensate for acceleration limits at each timestep, starting near the beginning of the trajectory.
  // Do not want to affect user-provided acceleration at the first timestep, so start at index 2.
  // Also do not want to affect user-provided acceleration at the last timestep.
  position_error = 0;
  double temp_accel = 0;
  for (size_t index = 1; index < (num_waypoints - 1); ++index)
  {
    if (fabs(waypoints_.accelerations(index)) > acceleration_limit)
    {
      delta_a =
          std::copysign(1.0, waypoints_.accelerations(index)) * acceleration_limit - waypoints_.accelerations(index);
      temp_accel = std::copysign(1.0, waypoints_.jerks(index)) * acceleration_limit;

      // Check jerk limit before applying the change.
      // The first condition checks if the new jerk(i) is going to exceed the limit. Pretty straightforward.
      // We also calculate a new jerk(i+1). The second condition checks if jerk(i+1) would exceed the limit.
      if (fabs(waypoints_.jerks(index) + delta_a / kTimestep) <= jerk_limit &&
          fabs((temp_accel - waypoints_.accelerations(index)) / kTimestep) <= jerk_limit)
      {
        waypoints_.accelerations(index) = temp_accel;
        waypoints_.jerks(index) = (waypoints_.accelerations(index) - waypoints_.accelerations(index - 1)) / kTimestep;
        // Recalculate jerk(i+1) to keep accel(i+1) and vel(i+1) the same
        waypoints_.jerks(index + 1) =
            (waypoints_.accelerations(index + 1) - waypoints_.accelerations(index)) / kTimestep;
      }
      else
      {
        // Acceleration and jerk limits cannot both be satisfied
        RecordFailureTime(index, index_last_successful);
        // Only break, do not return, because we are looking for the FIRST failure. May find an earlier failure in
        // subsequent code
        break;
      }

      delta_v = delta_a * kTimestep;
      waypoints_.velocities(index) = waypoints_.velocities(index) + delta_v;
      // Try decreasing the velocity in previous timesteps to compensate for
      // this limit, if needed
      successful_compensation = VelocityCompensation(index, delta_v);
      if (!successful_compensation)
      {
        position_error = position_error + delta_v * kTimestep;
      }
      if (fabs(position_error) > kPositionTolerance)
      {
        RecordFailureTime(index, index_last_successful);
        // Only break, do not return, because we are looking for the FIRST failure. May find an earlier failure in
        // subsequent code
        break;
      }
    }
  }

  // Compensate for velocity limits at each timestep, starting near the beginning of the trajectory.
  // Do not want to affect user-provided velocity at the first timestep, so start at index 2.
  // Also do not want to affect user-provided velocity at the last timestep.
  position_error = 0;
  for (size_t index = 1; index < (num_waypoints - 1); ++index)
  {
    delta_v = 0;

    // If the velocity limit would be exceeded
    if (fabs(waypoints_.velocities(index)) > velocity_limit)
    {
      delta_v = std::copysign(1.0, waypoints_.velocities(index)) * velocity_limit - waypoints_.velocities(index);
      waypoints_.velocities(index) = std::copysign(1.0, waypoints_.velocities(index)) * velocity_limit;
      waypoints_.accelerations(index) = (waypoints_.velocities(index) - waypoints_.velocities(index - 1)) / kTimestep;
      waypoints_.jerks(index) = (waypoints_.accelerations(index) - waypoints_.accelerations(index - 1)) / kTimestep;

      // If a velocity adjustment was made
      if (delta_v != 0)
      {
        // Try decreasing the velocity in previous timesteps to compensate for this limit
        successful_compensation = VelocityCompensation(index, delta_v);
        if (!successful_compensation)
        {
          position_error = position_error + delta_v * kTimestep;
        }
        if (fabs(position_error) > kPositionTolerance || fabs(waypoints_.accelerations(index)) > acceleration_limit ||
            fabs(waypoints_.jerks(index)) > jerk_limit ||
            fabs(waypoints_.accelerations(index + 1)) > acceleration_limit ||
            fabs(waypoints_.jerks(index + 1)) > jerk_limit)
        {
          RecordFailureTime(index, index_last_successful);
          // Only break, do not return, because we are looking for the FIRST failure. May find an earlier failure in
          // subsequent code
          break;
        }
      }

      // If possible, adjust velocity in the next timestep to account for any position error.
      if (position_error)
      {
        // Velocity adjustment at timestep i+1
        double vel_to_cancel_error = waypoints_.velocities(index + 1) + position_error / kTimestep;
        double accel_to_cancel_error = (vel_to_cancel_error - waypoints_.velocities(index)) / kTimestep;
        double jerk_to_cancel_error = (accel_to_cancel_error - waypoints_.accelerations(index)) / kTimestep;

        // If vel(i+1) changes, need to re-check accel(i+2) and jerk(i+2)
        double accel2 = 0;  // New acceleration at i+2
        double jerk2 = 0;   // New jerk at i+2
        if (index < num_waypoints - 1)
        {
          accel2 = (waypoints_.velocities(index + 2) - vel_to_cancel_error) / kTimestep;
          jerk2 = (accel2 - accel_to_cancel_error) / kTimestep;
        }

        // Apply the new velocity unless it would violate limits
        if (fabs(vel_to_cancel_error) <= velocity_limit && fabs(accel_to_cancel_error) <= acceleration_limit &&
            fabs(jerk_to_cancel_error) <= jerk_limit && fabs(accel2) <= acceleration_limit && fabs(jerk2) <= jerk_limit)
        {
          waypoints_.velocities(index + 1) = vel_to_cancel_error;
          waypoints_.accelerations(index + 1) = accel_to_cancel_error;
          waypoints_.jerks(index + 1) = jerk_to_cancel_error;
          position_error = 0;
        }
      }
    }
  }
  return ErrorCodeEnum::kNoError;
}

void SingleJointGenerator::RecordFailureTime(size_t current_index, size_t* index_last_successful)
{
  // Record the index when compensation first failed
  if (current_index < *index_last_successful)
  {
    *index_last_successful = current_index;
  }
}

bool SingleJointGenerator::VelocityCompensation(size_t limited_index, double excess_velocity)
{
  // The algorithm:
  // 1) check jerk limits, from beginning to end of trajectory. Don't bother
  // checking accel/vel limits here, they will be checked next
  // 2) check accel limits. Make sure it doesn't cause jerk to exceed limits.
  // 3) check vel limits. This will also check whether previously-checked
  // jerk/accel limits were exceeded

  bool successful_compensation = false;

  // Add a bit of velocity at step i to compensate for the limit at timestep
  // i+1.
  // Cannot go beyond index 2 because we use a 2-index window for derivative
  // calculations.
  for (size_t index = limited_index; index > 2; --index)
  {
    // if there is some room to increase the velocity at timestep i
    if (fabs(waypoints_.velocities(index)) < kLimits.velocity_limit)
    {
      // If the full change can be made in this timestep
      if ((excess_velocity > 0 && waypoints_.velocities(index) <= kLimits.velocity_limit - excess_velocity) ||
          (excess_velocity < 0 && waypoints_.velocities(index) >= -kLimits.velocity_limit - excess_velocity))
      {
        double new_velocity = waypoints_.velocities(index) + excess_velocity;
        // Accel and jerk, calculated from the previous waypoints
        double backward_accel = (new_velocity - waypoints_.velocities(index - 1)) / kTimestep;
        double backward_jerk =
            (backward_accel - (waypoints_.velocities(index - 1) - waypoints_.velocities(index - 2)) / kTimestep) /
            kTimestep;
        // Accel and jerk, calculated from upcoming waypoints
        double forward_accel = (waypoints_.velocities(index + 1) - new_velocity) / kTimestep;
        double forward_jerk =
            (forward_accel - (new_velocity - waypoints_.velocities(index - 1)) / kTimestep) / kTimestep;

        // Calculate this new velocity if it would not violate accel/jerk limits
        if ((fabs(backward_jerk) < kLimits.jerk_limit) && (fabs(backward_accel) < kLimits.acceleration_limit) &&
            (fabs(forward_jerk) < kLimits.jerk_limit) && (fabs(forward_accel) < kLimits.acceleration_limit))
        {
          waypoints_.velocities(index) = new_velocity;

          // if at the velocity limit, must stop accelerating
          if (fabs(waypoints_.velocities(index)) >= kLimits.velocity_limit)
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
      // else, can't make all of the correction in this timestep, so make as
      // much of a change as possible
      else
      {
        // This is what accel and jerk would be if we set velocity(index) to the
        // limit
        double new_velocity = std::copysign(1.0, excess_velocity) * kLimits.velocity_limit;
        // Accel and jerk, calculated from the previous waypoints
        double backward_accel = (new_velocity - waypoints_.velocities(index - 1)) / kTimestep;
        double backward_jerk =
            (backward_accel - (waypoints_.velocities(index - 1) - waypoints_.velocities(index - 2)) / kTimestep) /
            kTimestep;
        // Accel and jerk, calculated from upcoming waypoints
        double forward_accel = (waypoints_.velocities(index + 1) - new_velocity) / kTimestep;
        double forward_jerk =
            (forward_accel - (new_velocity - waypoints_.velocities(index - 1)) / kTimestep) / kTimestep;

        // Apply these changes if all limits are satisfied (for the prior
        // waypoint and the future waypoint)
        if ((fabs(backward_accel) < kLimits.acceleration_limit) && (fabs(backward_jerk) < kLimits.jerk_limit) &&
            (fabs(forward_accel) < kLimits.acceleration_limit) && (fabs(forward_jerk) < kLimits.jerk_limit))
        {
          double delta_v = new_velocity - waypoints_.velocities(index);
          waypoints_.velocities(index) = new_velocity;

          // if at the velocity limit, must stop accelerating
          if (fabs(waypoints_.velocities(index)) >= kLimits.velocity_limit)
          {
            waypoints_.accelerations(index) = 0;
            waypoints_.jerks(index) = 0;
          }
          else
          {
            waypoints_.accelerations(index) = backward_accel;
            waypoints_.jerks(index) = backward_jerk;
          }
          excess_velocity = excess_velocity - delta_v;
        }
      }
    }
    // Clip velocities at min/max due to small rounding errors
    waypoints_.velocities(index) =
        std::min(std::max(waypoints_.velocities(index), -kLimits.velocity_limit), kLimits.velocity_limit);
  }

  return successful_compensation;
}

ErrorCodeEnum SingleJointGenerator::PredictTimeToReach()
{
  // Take a trajectory that could not reach the desired position in time.
  // Try increasing the duration until it is interpolated without violating
  // limits.
  // This gives a new duration estimate.

  ErrorCodeEnum error_code = ErrorCodeEnum::kNoError;

  // Iterate over new durations until the position error is acceptable or the
  // maximum duration is reached
  while ((index_last_successful_ < static_cast<size_t>(waypoints_.positions.size())) &&
         (desired_duration_ < max_duration_))
  {
    // Try increasing the duration, based on fraction of states that weren't
    // reached successfully
    // TODO(andyz): Ensure at least one new timestep is added
    desired_duration_ = (1. + 0.5 * (1. - index_last_successful_ / waypoints_.positions.size())) * desired_duration_;

    // // Round to nearest timestep
    if (std::fmod(desired_duration_, kTimestep) > 0.5 * kTimestep)
      desired_duration_ = desired_duration_ + kTimestep;

    size_t new_num_waypoints = floor(1 + desired_duration_ / kTimestep);
    // Cap the trajectory duration to maintain determinism
    if (new_num_waypoints > kMaxNumWaypoints)
      new_num_waypoints = kMaxNumWaypoints;

    waypoints_.elapsed_times.setLinSpaced(new_num_waypoints, 0., desired_duration_);
    waypoints_.positions.resize(waypoints_.elapsed_times.size());
    waypoints_.velocities.resize(waypoints_.elapsed_times.size());
    waypoints_.accelerations.resize(waypoints_.elapsed_times.size());
    waypoints_.jerks.resize(waypoints_.elapsed_times.size());

    ////////////////////////////////////////////////////////////
    // Try to create the trajectory again, with the new duration
    ////////////////////////////////////////////////////////////
    waypoints_.positions = Interpolate(waypoints_.elapsed_times);
    CalculateDerivatives();
    PositionVectorLimitLookAhead(&index_last_successful_);
  }

  // Error if we extended the duration to the maximum and it still wasn't
  // successful
  if (index_last_successful_ < static_cast<size_t>(waypoints_.elapsed_times.size()))
  {
    error_code = ErrorCodeEnum::kMaxDurationExceeded;
  }

  return error_code;
}

ErrorCodeEnum SingleJointGenerator::PositionVectorLimitLookAhead(size_t* index_last_successful)
{
  ErrorCodeEnum error_code = LimitCompensation(index_last_successful);
  if (error_code)
    return error_code;

  // Re-compile the position with these modifications.
  // Ensure the first and last elements are a perfect match with initial/final
  // conditions
  double kOneSixth = 0.166667;
  // Initial waypoint
  waypoints_.positions(0) = kCurrentJointState.position;
  for (size_t index = 1; index < static_cast<size_t>(waypoints_.positions.size()) - 1; ++index)
    waypoints_.positions(index) = waypoints_.positions(index - 1) + waypoints_.velocities(index - 1) * kTimestep +
                                  0.5 * waypoints_.accelerations(index - 1) * pow(kTimestep, 2) +
                                  kOneSixth * waypoints_.jerks(index - 1) * pow(kTimestep, 3);
  // Final waypoint
  waypoints_.positions(waypoints_.positions.size() - 1) = kGoalJointState.position;

  return error_code;
}

void SingleJointGenerator::CalculateDerivatives()
{
  // From position vector, approximate velocity and acceleration.
  // velocity = (difference between adjacent position elements) / delta_t
  // acceleration = (difference between adjacent velocity elements) / delta_t
  waypoints_.velocities = DiscreteDifferentiation(waypoints_.positions, kTimestep, kCurrentJointState.velocity);
  waypoints_.accelerations = DiscreteDifferentiation(waypoints_.velocities, kTimestep, kCurrentJointState.acceleration);
  waypoints_.jerks = DiscreteDifferentiation(waypoints_.accelerations, kTimestep, 0);

  // Ensure the initial conditions and final positions are exactly copied
  // TODO(andyz)
}

void SingleJointGenerator::UpdateTrajectoryDuration(double new_trajectory_duration)
{
  // The trajectory will be forced to have this duration (or fail) because
  // max_duration == desired_duration
  desired_duration_ = new_trajectory_duration;
  max_duration_ = new_trajectory_duration;
}
}  // end namespace trackjoint
