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
SingleJointGenerator::SingleJointGenerator(double timestep, double desired_duration, double max_duration,
                                           const trackjoint::KinematicState& current_joint_state,
                                           const trackjoint::KinematicState& goal_joint_state,
                                           const trackjoint::Limits& limits, size_t desired_num_waypoints,
                                           size_t min_num_waypoints, size_t max_num_waypoints,
                                           const double position_tolerance, bool use_high_speed_mode)
  : kMaxNumHighSpeedWaypoints(min_num_waypoints)
  , kMaxNumWaypoints(max_num_waypoints)
  , timestep_(timestep)
  , desired_duration_(desired_duration)
  , max_duration_(max_duration)
  , current_joint_state_(current_joint_state)
  , goal_joint_state_(goal_joint_state)
  , limits_(limits)
  , position_tolerance_(position_tolerance)
  , use_high_speed_mode_(use_high_speed_mode)
{
  // Waypoint times
  nominal_times_ = Eigen::VectorXd::LinSpaced(desired_num_waypoints, 0, desired_duration_);
}

void SingleJointGenerator::Reset(double timestep, double desired_duration, double max_duration,
      const KinematicState& current_joint_state, const KinematicState& goal_joint_state,
      const trackjoint::Limits& limits, size_t desired_num_waypoints, const double position_tolerance,
      bool use_high_speed_mode)
{
  timestep_ = timestep;
  desired_duration_ = desired_duration;
  max_duration_ = max_duration;
  current_joint_state_ = current_joint_state;
  goal_joint_state_ = goal_joint_state;
  limits_ = limits;
  position_tolerance_ = position_tolerance;
  use_high_speed_mode_ = use_high_speed_mode;

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
  size_t expected_num_waypoints = 1 + desired_duration_ / timestep_;
  waypoints_.elapsed_times.setLinSpaced(expected_num_waypoints, 0., desired_duration_);
  waypoints_.positions = Interpolate(waypoints_.elapsed_times);
  CalculateDerivatives();
  ErrorCodeEnum error_code = ForwardLimitCompensation(&index_last_successful_);
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

inline Eigen::VectorXd SingleJointGenerator::Interpolate(Eigen::VectorXd& times)
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

inline ErrorCodeEnum SingleJointGenerator::ForwardLimitCompensation(size_t* index_last_successful)
{
  // This is the indexing convention.
  // 1. accel(i) = accel(i-1) + jerk(i) * dt
  // 2. vel(i) == vel(i-1) + accel(i-1) * dt + 0.5 * jerk(i) * dt ^ 2

  // Start with the assumption that the entire trajectory can be completed
  // High-speed mode returns at the minimum number of waypoints.
  if (!use_high_speed_mode_)
    *index_last_successful = waypoints_.positions.size();
  else
    *index_last_successful = kMaxNumHighSpeedWaypoints;

  bool successful_compensation = false;

  // Discrete differentiation introduces small numerical errors, so allow a small tolerance
  const double kLimitRelativeTol = 0.999999;
  const double kJerkLimit = kLimitRelativeTol * limits_.jerk_limit;
  const double kAccelerationLimit = kLimitRelativeTol * limits_.acceleration_limit;
  const double kVelocityLimit = kLimitRelativeTol * limits_.velocity_limit;

  // Preallocate
  double delta_a(0), delta_v(0), position_error(0);

  // If in high-speed mode, return as fast as possible (minimum num. waypoints)
  // Subtract one because we do not want to affect vel/accel at the last timestep
  std::size_t last_waypoint_to_adjust;
  // High speed mode is not necessary if the number of waypoints is already very short
  if (!use_high_speed_mode_ || static_cast<size_t>(waypoints_.positions.size()) <= kMaxNumHighSpeedWaypoints)
    last_waypoint_to_adjust = waypoints_.positions.size() - 1;
  // Shorten the trajectory, if in high-speed mode
  else
  {
    last_waypoint_to_adjust = kMaxNumHighSpeedWaypoints - 1;
  }

  // Compensate for jerk limits at each timestep, starting near the beginning
  // Do not want to affect vel/accel at the first/last timestep
  for (size_t index = 1; index < last_waypoint_to_adjust; ++index)
  {
    if (fabs(waypoints_.jerks(index)) > kJerkLimit)
    {
      waypoints_.jerks(index) = std::copysign(kJerkLimit, waypoints_.jerks(index));
      double delta_j = waypoints_.jerks(index) - waypoints_.jerks(index);

      waypoints_.accelerations(index) = waypoints_.accelerations(index - 1) + waypoints_.jerks(index) * timestep_;
      waypoints_.velocities(index) = waypoints_.velocities(index - 1) +
                                     waypoints_.accelerations(index - 1) * timestep_ +
                                     0.5 * waypoints_.jerks(index) * timestep_ * timestep_;

      // Re-calculate derivatives from the updated velocity vector
      CalculateDerivatives();

      delta_v = 0.5 * delta_j * timestep_ * timestep_;

      // Try adjusting the velocity in previous timesteps to compensate for this limit, if needed
      successful_compensation = BackwardLimitCompensation(index, &delta_v);
      if (!successful_compensation)
      {
        position_error = position_error + delta_v * timestep_;
      }
      if (fabs(position_error) > position_tolerance_)
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
  for (size_t index = 1; index < last_waypoint_to_adjust; ++index)
  {
    if (fabs(waypoints_.accelerations(index)) > kAccelerationLimit)
    {
      double temp_accel = std::copysign(kAccelerationLimit, waypoints_.accelerations(index));
      delta_a = temp_accel - waypoints_.accelerations(index);

      // Check jerk limit before applying the change.
      // The first condition checks if the new jerk(i) is going to exceed the limit. Pretty straightforward.
      // We also calculate a new jerk(i+1). The second condition checks if jerk(i+1) would exceed the limit.
      if ((fabs((temp_accel - waypoints_.accelerations(index)) / timestep_) <= kJerkLimit) &&
          (fabs(waypoints_.jerks(index) + delta_a / timestep_) <= kJerkLimit))
      {
        waypoints_.accelerations(index) = temp_accel;
        waypoints_.jerks(index) = (waypoints_.accelerations(index) - waypoints_.accelerations(index - 1)) / timestep_;
        waypoints_.velocities(index) = waypoints_.velocities(index - 1) +
                                       waypoints_.accelerations(index - 1) * timestep_ +
                                       0.5 * waypoints_.jerks(index) * timestep_ * timestep_;
        // Re-calculate derivatives from the updated velocity vector
        CalculateDerivatives();
      }
      else
      {
        // Acceleration and jerk limits cannot both be satisfied
        RecordFailureTime(index, index_last_successful);
        // Only break, do not return, because we are looking for the FIRST failure. May find an earlier failure in
        // subsequent code
        break;
      }

      // Try adjusting the velocity in previous timesteps to compensate for this limit, if needed
      delta_v = delta_a * timestep_;
      successful_compensation = BackwardLimitCompensation(index, &delta_v);
      if (!successful_compensation)
      {
        position_error = position_error + delta_v * timestep_;
      }
      if (fabs(position_error) > position_tolerance_)
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
  for (size_t index = 1; index < last_waypoint_to_adjust; ++index)
  {
    delta_v = 0;

    // If the velocity limit would be exceeded
    if (fabs(waypoints_.velocities(index)) > kVelocityLimit)
    {
      waypoints_.velocities(index) = std::copysign(kVelocityLimit, waypoints_.velocities(index));
      delta_v = waypoints_.velocities(index) - waypoints_.velocities(index);
      // Re-calculate derivatives from the updated velocity vector
      CalculateDerivatives();

      // Try adjusting the velocity in previous timesteps to compensate for this limit.
      // Try to account for position error, too.
      delta_v += position_error / timestep_;
      successful_compensation = BackwardLimitCompensation(index, &delta_v);
      if (!successful_compensation)
      {
        position_error = position_error + delta_v * timestep_;
      }
      else
        position_error = 0;

      if (fabs(position_error) > position_tolerance_ || fabs(waypoints_.accelerations(index)) > kAccelerationLimit ||
          fabs(waypoints_.jerks(index)) > kJerkLimit ||
          fabs(waypoints_.accelerations(index + 1)) > kAccelerationLimit ||
          fabs(waypoints_.jerks(index + 1)) > kJerkLimit)
      {
        RecordFailureTime(index, index_last_successful);
        // Only break, do not return, because we are looking for the FIRST failure. May find an earlier failure in
        // subsequent code
        break;
      }
    }
  }

  return ErrorCodeEnum::kNoError;
}

inline void SingleJointGenerator::RecordFailureTime(size_t current_index, size_t* index_last_successful)
{
  // Record the index when compensation first failed
  if (current_index < *index_last_successful)
  {
    *index_last_successful = current_index;
  }
}

inline bool SingleJointGenerator::BackwardLimitCompensation(size_t limited_index, double* excess_velocity)
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
    if (fabs(waypoints_.velocities(index)) < limits_.velocity_limit)
    {
      // If the full change can be made in this timestep
      if ((*excess_velocity > 0 && waypoints_.velocities(index) <= limits_.velocity_limit - *excess_velocity) ||
          (*excess_velocity < 0 && waypoints_.velocities(index) >= -limits_.velocity_limit - *excess_velocity))
      {
        double new_velocity = waypoints_.velocities(index) + *excess_velocity;
        // Accel and jerk, calculated from the previous waypoints
        double backward_accel = (new_velocity - waypoints_.velocities(index - 1)) / timestep_;
        double backward_jerk =
            (backward_accel - (waypoints_.velocities(index - 1) - waypoints_.velocities(index - 2)) / timestep_) /
            timestep_;
        // Accel and jerk, calculated from upcoming waypoints
        double forward_accel = (waypoints_.velocities(index + 1) - new_velocity) / timestep_;
        double forward_jerk =
            (forward_accel - (new_velocity - waypoints_.velocities(index - 1)) / timestep_) / timestep_;

        // Calculate this new velocity if it would not violate accel/jerk limits
        if ((fabs(backward_jerk) < limits_.jerk_limit) && (fabs(backward_accel) < limits_.acceleration_limit) &&
            (fabs(forward_jerk) < limits_.jerk_limit) && (fabs(forward_accel) < limits_.acceleration_limit))
        {
          waypoints_.velocities(index) = new_velocity;

          // if at the velocity limit, must stop accelerating
          if (fabs(waypoints_.velocities(index)) >= limits_.velocity_limit)
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
        double new_velocity = std::copysign(1.0, *excess_velocity) * limits_.velocity_limit;
        // Accel and jerk, calculated from the previous waypoints
        double backward_accel = (new_velocity - waypoints_.velocities(index - 1)) / timestep_;
        double backward_jerk =
            (backward_accel - (waypoints_.velocities(index - 1) - waypoints_.velocities(index - 2)) / timestep_) /
            timestep_;
        // Accel and jerk, calculated from upcoming waypoints
        double forward_accel = (waypoints_.velocities(index + 1) - new_velocity) / timestep_;
        double forward_jerk =
            (forward_accel - (new_velocity - waypoints_.velocities(index - 1)) / timestep_) / timestep_;

        // Apply these changes if all limits are satisfied (for the prior
        // waypoint and the future waypoint)
        if ((fabs(backward_accel) < limits_.acceleration_limit) && (fabs(backward_jerk) < limits_.jerk_limit) &&
            (fabs(forward_accel) < limits_.acceleration_limit) && (fabs(forward_jerk) < limits_.jerk_limit))
        {
          double delta_v = new_velocity - waypoints_.velocities(index);
          waypoints_.velocities(index) = new_velocity;

          // if at the velocity limit, must stop accelerating
          if (fabs(waypoints_.velocities(index)) >= limits_.velocity_limit)
          {
            waypoints_.accelerations(index) = 0;
            waypoints_.jerks(index) = 0;
          }
          else
          {
            waypoints_.accelerations(index) = backward_accel;
            waypoints_.jerks(index) = backward_jerk;
          }
          *excess_velocity = *excess_velocity - delta_v;
        }
      }
    }
    // Clip velocities at min/max due to small rounding errors
    waypoints_.velocities(index) =
        std::min(std::max(waypoints_.velocities(index), -limits_.velocity_limit), limits_.velocity_limit);
  }

  return successful_compensation;
}

inline ErrorCodeEnum SingleJointGenerator::PredictTimeToReach()
{
  // Take a trajectory that could not reach the desired position in time.
  // Try increasing the duration until it is interpolated without violating
  // limits.
  // This gives a new duration estimate.

  ErrorCodeEnum error_code = ErrorCodeEnum::kNoError;

  // If in normal mode, we can extend trajectories
  if (!use_high_speed_mode_)
  {
    size_t new_num_waypoints = 0;
    // Iterate over new durations until the position error is acceptable or the maximum duration is reached
    while ((index_last_successful_ < static_cast<size_t>(waypoints_.positions.size())) &&
           (desired_duration_ < max_duration_) && (new_num_waypoints < kMaxNumWaypoints))
    {
      // Try increasing the duration, based on fraction of states that weren't reached successfully
      desired_duration_ = (1. + 0.5 * (1. - index_last_successful_ / waypoints_.positions.size())) * desired_duration_;

      // // Round to nearest timestep
      if (std::fmod(desired_duration_, timestep_) > 0.5 * timestep_)
        desired_duration_ = desired_duration_ + timestep_;

      new_num_waypoints = std::max(static_cast<size_t>(waypoints_.positions.size() + 1),
                                   static_cast<size_t>(floor(1 + desired_duration_ / timestep_)));
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
  }
  // If in high-speed mode, do not extend the trajectories.
  // May need to clip the trajectories if only a few waypoints were successful.
  else
  {
    // Clip at the last successful index
    if (index_last_successful_ < kMaxNumHighSpeedWaypoints - 1)
    {
      // If in high-speed mode, clip at the shorter number of waypoints
      ClipEigenVector(&waypoints_.positions, index_last_successful_ + 1);
      ClipEigenVector(&waypoints_.velocities, index_last_successful_ + 1);
      ClipEigenVector(&waypoints_.accelerations, index_last_successful_ + 1);
      ClipEigenVector(&waypoints_.jerks, index_last_successful_ + 1);
      ClipEigenVector(&waypoints_.elapsed_times, index_last_successful_ + 1);
      // Eigen vectors do not have a "back" member function
      goal_joint_state_.position = waypoints_.positions[index_last_successful_];
      goal_joint_state_.velocity = waypoints_.velocities[index_last_successful_];
      goal_joint_state_.acceleration = waypoints_.accelerations[index_last_successful_];
      desired_duration_ = waypoints_.elapsed_times[index_last_successful_];
    }
    // else, clip at kMaxNumHighSpeedWaypoints
    else
    {
      // If in high-speed mode, clip at the shorter number of waypoints
      ClipEigenVector(&waypoints_.positions, kMaxNumHighSpeedWaypoints);
      ClipEigenVector(&waypoints_.velocities, kMaxNumHighSpeedWaypoints);
      ClipEigenVector(&waypoints_.accelerations, kMaxNumHighSpeedWaypoints);
      ClipEigenVector(&waypoints_.jerks, kMaxNumHighSpeedWaypoints);
      ClipEigenVector(&waypoints_.elapsed_times, kMaxNumHighSpeedWaypoints);
      // Eigen vectors do not have a "back" member function
      goal_joint_state_.position = waypoints_.positions[kMaxNumHighSpeedWaypoints - 1];
      goal_joint_state_.velocity = waypoints_.velocities[kMaxNumHighSpeedWaypoints - 1];
      goal_joint_state_.acceleration = waypoints_.accelerations[kMaxNumHighSpeedWaypoints - 1];
      desired_duration_ = waypoints_.elapsed_times[kMaxNumHighSpeedWaypoints - 1];
    }
  }

  // Normal mode: Error if we extended the duration to the maximum and it still wasn't successful
  if (!use_high_speed_mode_ && index_last_successful_ < static_cast<size_t>(waypoints_.elapsed_times.size()))
  {
    error_code = ErrorCodeEnum::kMaxDurationExceeded;
  }
  // Error if not even a single waypoint could be generated
  if (waypoints_.positions.size() < 2)
  {
    error_code = ErrorCodeEnum::kFailureToGenerateSingleWaypoint;
  }

  return error_code;
}

inline ErrorCodeEnum SingleJointGenerator::PositionVectorLimitLookAhead(size_t* index_last_successful)
{
  ErrorCodeEnum error_code = ForwardLimitCompensation(index_last_successful);
  if (error_code)
    return error_code;

  // Re-compile the position with these modifications.
  // Ensure the first and last elements are a perfect match with initial/final
  // conditions
  double kOneSixth = 0.166667;
  // Initial waypoint
  waypoints_.positions(0) = current_joint_state_.position;
  for (size_t index = 1; index < static_cast<size_t>(waypoints_.positions.size()) - 1; ++index)
    waypoints_.positions(index) = waypoints_.positions(index - 1) + waypoints_.velocities(index - 1) * timestep_ +
                                  0.5 * waypoints_.accelerations(index - 1) * pow(timestep_, 2) +
                                  kOneSixth * waypoints_.jerks(index - 1) * pow(timestep_, 3);

  // Final waypoint should match the goal, unless trajectory was cut short for high-speed mode
  if (!use_high_speed_mode_)
    waypoints_.positions(waypoints_.positions.size() - 1) = goal_joint_state_.position;

  return error_code;
}

inline void SingleJointGenerator::CalculateDerivatives()
{
  // From position vector, approximate velocity and acceleration.
  // velocity = (difference between adjacent position elements) / delta_t
  // acceleration = (difference between adjacent velocity elements) / delta_t
  waypoints_.velocities = DiscreteDifferentiation(waypoints_.positions, timestep_, current_joint_state_.velocity);
  waypoints_.accelerations = DiscreteDifferentiation(waypoints_.velocities, timestep_, current_joint_state_.acceleration);
  waypoints_.jerks = DiscreteDifferentiation(waypoints_.accelerations, timestep_, 0);
}

void SingleJointGenerator::UpdateTrajectoryDuration(double new_trajectory_duration)
{
  // The trajectory will be forced to have this duration (or fail) because
  // max_duration == desired_duration
  desired_duration_ = new_trajectory_duration;
  max_duration_ = new_trajectory_duration;
}
}  // end namespace trackjoint
