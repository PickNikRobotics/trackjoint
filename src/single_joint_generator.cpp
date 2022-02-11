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

#include "trackjoint/single_joint_generator.h"

namespace trackjoint
{
namespace
{
constexpr long double DEFAULT_FILTER_COEFFICIENT = 2.0;
}

SingleJointGenerator::SingleJointGenerator(size_t num_waypoints_threshold, size_t max_num_waypoints_trajectory_mode)
  : kNumWaypointsThreshold(num_waypoints_threshold)
  , kMaxNumWaypointsFullTrajectory(max_num_waypoints_trajectory_mode)
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
  index_last_successful_ = 0;
  is_reset_ = true;

  // Start with this estimate of the shortest possible duration
  // The shortest possible duration avoids oscillation, as much as possible
  // Desired duration cannot be less than one timestep
  if (!timestep_was_upsampled)
  {
    desired_duration_ =
        std::max(configuration.timestep, std::fabs((goal_joint_state.position - current_joint_state.position) /
                                              configuration_.limits.velocity_limit));
  }
  // If upsampling was used, we don't want to mess with the timestep or duration minimization
  else
  {
    desired_duration_ = (desired_num_waypoints - 1) * configuration.timestep;
  }

  // Waypoint times
  nominal_times_ = VectorXlong::LinSpaced(desired_num_waypoints, 0, desired_duration_);
}

void SingleJointGenerator::reset(const long double timestep, const long double max_duration, const KinematicState& current_joint_state,
                                 const KinematicState& goal_joint_state, const Limits& limits,
                                 size_t desired_num_waypoints, const long double position_tolerance, bool use_streaming_mode,
                                 bool timestep_was_upsampled)
{
  Configuration configuration(timestep, max_duration, limits, position_tolerance, use_streaming_mode);
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

  ErrorCodeEnum error_code = positionVectorLimitLookAhead(&index_last_successful_);
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
  const size_t orig_num_waypoints = waypoints_.elapsed_times.size();
  size_t new_num_waypoints = 1 + desired_duration_ / configuration_.timestep;

  // If waypoints were successfully generated for this dimension previously, just stretch the trajectory with splines.
  // ^This is the best way because it reduces overshoot.
  // Otherwise, re-generate a new trajectory from scratch.
  if (index_last_successful_ == static_cast<size_t>(waypoints_.elapsed_times.size() - 1))
  {
    // Fit a spline function to the original positions but new (extended) duration

    // The algorithm:
    // Fit a spline function to the original waypoints, same number of waypoints, new (stretched) timesteps
    // Ensure slopes at timestep(0:1) and timestep(end-1 : end) do not change (initial and goal slopes)
    // Re-sample positions from the spline at the correct delta-t
    // Run forwardLimitCompensation() to ensure limits are obeyed
    Eigen::Matrix<long double, 1, Eigen::Dynamic> stretched_times = Eigen::Matrix<long double, 1, Eigen::Dynamic>::Zero(1, orig_num_waypoints);
    // Linearly increase the time stretch, from zero stretch at the beginning to max stretch in the middle.
    // Do the same for ramping down.
    // This ends up with an average stretch as desired.
    // We do not stretch either end so that initial slope & final slope do not change
    long double net_stretch = desired_duration_ / waypoints_.elapsed_times(orig_num_waypoints - 1);
    long double stretch_factor = 0;
    for (size_t timestep_idx = 1; timestep_idx < orig_num_waypoints; ++timestep_idx)
    {
      if (timestep_idx <= orig_num_waypoints / 2.)
      {
        stretch_factor =
            1 + 4. * (net_stretch - 1) * static_cast<long double>(timestep_idx) / static_cast<long double>(orig_num_waypoints);
      }
      else
      {
        stretch_factor =
            ((4 - 4 * net_stretch) / static_cast<long double>(orig_num_waypoints)) * static_cast<long double>(timestep_idx) +
            4. * net_stretch - 3. - 2. / static_cast<long double>(orig_num_waypoints);
      }
      stretched_times(timestep_idx) = stretched_times(timestep_idx - 1) + configuration_.timestep * stretch_factor;
    }
    // Final time is the new desired_duration_
    stretched_times(orig_num_waypoints - 1) = desired_duration_;

    Eigen::Matrix<long double, 1, Eigen::Dynamic> position = Eigen::Matrix<long double, 1, Eigen::Dynamic>(waypoints_.positions);
    const auto fit = SplineFitting1D::Interpolate(position, 2, stretched_times);

    // New times, with the extended duration
    waypoints_.elapsed_times.setLinSpaced(new_num_waypoints, 0., (new_num_waypoints - 1) * configuration_.timestep);
    // Retrieve new positions at the new times
    waypoints_.positions.resize(new_num_waypoints);

    for (Eigen::Index idx = 0; idx < waypoints_.elapsed_times.size(); ++idx)
      waypoints_.positions[idx] = fit(waypoints_.elapsed_times(idx)).coeff(0);

    calculateDerivativesFromPosition();
    forwardLimitCompensation(&index_last_successful_);
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
    forwardLimitCompensation(&index_last_successful_);
  }

  return;
}

JointTrajectory SingleJointGenerator::getTrajectory()
{
  return waypoints_;
}

size_t SingleJointGenerator::getLastSuccessfulIndex()
{
  return index_last_successful_;
}

VectorXlong SingleJointGenerator::interpolate(VectorXlong& times)
{
  // See De Luca, "Trajectory Planning" pdf, slide 19
  // Interpolate a smooth trajectory from initial to final state while matching
  // boundary conditions.

  // De Luca uses tao to represent a normalized time
  VectorXlong tao = times / desired_duration_;

  // TODO(andyz): vectorize this calculation
  VectorXlong interpolated_position(tao.size());
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

ErrorCodeEnum SingleJointGenerator::forwardLimitCompensation(size_t* index_last_successful)
{
  // The algorithm:
  // 1) check jerk limits, from beginning to end of trajectory. Don't bother
  // checking accel/vel limits here, they will be checked next
  // 2) check accel limits. Make sure it doesn't cause jerk to exceed limits.
  // 3) check vel limits. This will also check whether previously-checked
  // jerk/accel limits were exceeded

  // This is the indexing convention.
  // 1. accel(i) = accel(i-1) + jerk(i) * dt
  // 2. vel(i) == vel(i-1) + accel(i-1) * dt + 0.5 * jerk(i) * dt ^ 2

  // Start with the assumption that the entire trajectory can be completed.
  // Streaming mode returns at the minimum number of waypoints.
  // Streaming mode is not necessary if the number of waypoints is already very short.
  if (!configuration_.use_streaming_mode || static_cast<size_t>(waypoints_.positions.size()) <= kNumWaypointsThreshold)
    *index_last_successful = waypoints_.positions.size() - 1;
  else
    *index_last_successful = kNumWaypointsThreshold - 1;

  bool successful_compensation = false;

  // Discrete differentiation introduces small numerical errors, so allow a small tolerance
  const long double limit_relative_tol = 0.999999;
  const long double jerk_limit = limit_relative_tol * configuration_.limits.jerk_limit;
  const long double acceleration_limit = limit_relative_tol * configuration_.limits.acceleration_limit;
  const long double velocity_limit = limit_relative_tol * configuration_.limits.velocity_limit;

  // Preallocate
  long double delta_a(0), delta_v(0), position_error(0);

  // Compensate for jerk limits at each timestep, starting near the beginning
  // Do not want to affect vel/accel at the first/last timestep
  for (size_t index = 1; index < *index_last_successful; ++index)
  {
    if (fabs(waypoints_.jerks(index)) > jerk_limit)
    {
      long double delta_j = std::copysign(jerk_limit, waypoints_.jerks(index)) - waypoints_.jerks(index);
      waypoints_.jerks(index) = std::copysign(jerk_limit, waypoints_.jerks(index));

      waypoints_.accelerations(index) =
          waypoints_.accelerations(index - 1) + waypoints_.jerks(index) * configuration_.timestep;
      waypoints_.velocities(index) = waypoints_.velocities(index - 1) +
                                     waypoints_.accelerations(index - 1) * configuration_.timestep +
                                     0.5 * waypoints_.jerks(index) * configuration_.timestep * configuration_.timestep;

      delta_v = 0.5 * delta_j * configuration_.timestep * configuration_.timestep;

      // Try adjusting the velocity in previous timesteps to compensate for this limit, if needed
      successful_compensation = backwardLimitCompensation(index, -delta_v);
      if (!successful_compensation)
      {
        position_error = position_error + delta_v * configuration_.timestep;
      }
      if (fabs(position_error) > configuration_.position_tolerance)
      {
        recordFailureTime(index, index_last_successful);
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
  for (size_t index = 1; index < *index_last_successful; ++index)
  {
    if (fabs(waypoints_.accelerations(index)) > acceleration_limit)
    {
      long double temp_accel = std::copysign(acceleration_limit, waypoints_.accelerations(index));
      delta_a = temp_accel - waypoints_.accelerations(index);

      // Check jerk limit before applying the change.
      // The first condition checks if the new jerk(i) is going to exceed the limit. Pretty straightforward.
      // We also calculate a new jerk(i+1). The second condition checks if jerk(i+1) would exceed the limit.
      if ((fabs((temp_accel - waypoints_.accelerations(index)) / configuration_.timestep) <= jerk_limit) &&
          (fabs(waypoints_.jerks(index) + delta_a / configuration_.timestep) <= jerk_limit))
      {
        waypoints_.accelerations(index) = temp_accel;
        waypoints_.jerks(index) =
            (waypoints_.accelerations(index) - waypoints_.accelerations(index - 1)) / configuration_.timestep;
        waypoints_.velocities(index) =
            waypoints_.velocities(index - 1) + waypoints_.accelerations(index - 1) * configuration_.timestep +
            0.5 * waypoints_.jerks(index) * configuration_.timestep * configuration_.timestep;
      }
      else
      {
        // Acceleration and jerk limits cannot both be satisfied
        recordFailureTime(index, index_last_successful);
        // Only break, do not return, because we are looking for the FIRST failure. May find an earlier failure in
        // subsequent code
        break;
      }

      // Try adjusting the velocity in previous timesteps to compensate for this limit, if needed
      delta_v = delta_a * configuration_.timestep;
      successful_compensation = backwardLimitCompensation(index, -delta_v);
      if (!successful_compensation)
      {
        position_error = position_error + delta_v * configuration_.timestep;
      }
      if (fabs(position_error) > configuration_.position_tolerance)
      {
        recordFailureTime(index, index_last_successful);
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
  for (size_t index = 1; index < *index_last_successful; ++index)
  {
    // If the velocity limit would be exceeded
    if (fabs(waypoints_.velocities(index)) > velocity_limit)
    {
      delta_v = std::copysign(velocity_limit, waypoints_.velocities(index)) - waypoints_.velocities(index);
      waypoints_.velocities(index) = std::copysign(velocity_limit, waypoints_.velocities(index));

      // Try adjusting the velocity in previous timesteps to compensate for this limit.
      // Try to account for position error, too.
      delta_v += position_error / configuration_.timestep;
      successful_compensation = backwardLimitCompensation(index, -delta_v);
      if (!successful_compensation)
      {
        position_error = position_error + delta_v * configuration_.timestep;
      }
      else
        position_error = 0;

      if (fabs(position_error) > configuration_.position_tolerance ||
          fabs(waypoints_.accelerations(index)) > acceleration_limit || fabs(waypoints_.jerks(index)) > jerk_limit ||
          fabs(waypoints_.accelerations(index + 1)) > acceleration_limit ||
          fabs(waypoints_.jerks(index + 1)) > jerk_limit)
      {
        recordFailureTime(index, index_last_successful);
        // Only break, do not return, because we are looking for the FIRST failure. May find an earlier failure in
        // subsequent code
        break;
      }
    }
  }

  // Re-calculate derivatives from the updated velocity vector
  calculateDerivativesFromVelocity();

  return ErrorCodeEnum::NO_ERROR;
}

bool SingleJointGenerator::backwardLimitCompensation(const size_t limited_index, long double excess_velocity)
{
  bool successful_compensation = false;

  // Since (index + 1) is used in some calculations below, the algorithm won't work if
  // limited_index is the last element in the array
  if (limited_index == (unsigned)waypoints_.velocities.size() - 1)
  {
    return false;
  }

  // Add a bit of velocity at step i to compensate for the limit at timestep i+1.
  // Cannot go beyond index 2 because we use a 2-index window for derivative calculations.
  for (size_t index = limited_index; index > 2; --index)
  {
    // if there is some room to increase the velocity at timestep i
    if (fabs(waypoints_.velocities(index)) < configuration_.limits.velocity_limit)
    {
      // If the full change can be made in this timestep
      if (((excess_velocity > 0) &&
           (waypoints_.velocities(index) <= configuration_.limits.velocity_limit - excess_velocity)) ||
          ((excess_velocity < 0) &&
           (waypoints_.velocities(index) >= -configuration_.limits.velocity_limit - excess_velocity)))
      {
        long double new_velocity = waypoints_.velocities(index) + excess_velocity;
        // Accel and jerk, calculated from the previous waypoints
        long double backward_accel = (new_velocity - waypoints_.velocities(index - 1)) / configuration_.timestep;
        long double backward_jerk = (backward_accel - (waypoints_.velocities(index - 1) - waypoints_.velocities(index - 2)) /
                                                     configuration_.timestep) /
                               configuration_.timestep;
        // Accel and jerk, calculated from upcoming waypoints
        long double forward_accel = (waypoints_.velocities(index + 1) - new_velocity) / configuration_.timestep;
        long double forward_jerk =
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
        long double new_velocity = std::copysign(1.0, excess_velocity) * configuration_.limits.velocity_limit;
        // Accel and jerk, calculated from the previous waypoints
        long double backward_accel = (new_velocity - waypoints_.velocities(index - 1)) / configuration_.timestep;
        long double backward_jerk = (backward_accel - (waypoints_.velocities(index - 1) - waypoints_.velocities(index - 2)) /
                                                     configuration_.timestep) /
                               configuration_.timestep;
        // Accel and jerk, calculated from upcoming waypoints
        long double forward_accel = (waypoints_.velocities(index + 1) - new_velocity) / configuration_.timestep;
        long double forward_jerk =
            (forward_accel - (new_velocity - waypoints_.velocities(index - 1)) / configuration_.timestep) /
            configuration_.timestep;

        // Apply these changes if all limits are satisfied (for the prior
        // waypoint and the future waypoint)
        if ((fabs(backward_accel) < configuration_.limits.acceleration_limit) &&
            (fabs(backward_jerk) < configuration_.limits.jerk_limit) &&
            (fabs(forward_accel) < configuration_.limits.acceleration_limit) &&
            (fabs(forward_jerk) < configuration_.limits.jerk_limit))
        {
          long double delta_v = new_velocity - waypoints_.velocities(index);
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

  // If in normal mode, we can extend trajectories
  if (!configuration_.use_streaming_mode)
  {
    size_t new_num_waypoints = 0;
    // Iterate over new durations until the position error is acceptable or the maximum duration is reached
    while ((index_last_successful_ < static_cast<size_t>(waypoints_.positions.size() - 1)) &&
           (desired_duration_ < configuration_.max_duration) && (new_num_waypoints < kMaxNumWaypointsFullTrajectory))
    {
      // Try increasing the duration, based on fraction of states that weren't reached successfully
      // Choice of 0.2 is subjective but it should be between 0-1.
      // A smaller fraction will find a solution that's closer to time-optimal because it adds fewer new waypoints to
      // the search. But, a smaller fraction likely increases runtime.
      desired_duration_ =
          (1. + 0.2 * (1. - index_last_successful_ / (waypoints_.positions.size() - 1))) * desired_duration_;

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
      positionVectorLimitLookAhead(&index_last_successful_);
    }
  }
  // If in streaming mode, do not extend the trajectories.
  // May need to clip the trajectories if only a few waypoints were successful.
  else
  {
    // Clip at the last successful index
    if (index_last_successful_ + 1 < kNumWaypointsThreshold)
    {
      // If in streaming mode, clip at the shorter number of waypoints
      clipEigenVector(&waypoints_.positions, index_last_successful_ + 1);
      clipEigenVector(&waypoints_.velocities, index_last_successful_ + 1);
      clipEigenVector(&waypoints_.accelerations, index_last_successful_ + 1);
      clipEigenVector(&waypoints_.jerks, index_last_successful_ + 1);
      clipEigenVector(&waypoints_.elapsed_times, index_last_successful_ + 1);
      // Eigen vectors do not have a "back" member function
      goal_joint_state_.position = waypoints_.positions[index_last_successful_];
      goal_joint_state_.velocity = waypoints_.velocities[index_last_successful_];
      goal_joint_state_.acceleration = waypoints_.accelerations[index_last_successful_];
      desired_duration_ = waypoints_.elapsed_times[index_last_successful_];
    }
    // else, clip at kNumWaypointsThreshold
    else
    {
      // If in streaming mode, clip at the shorter number of waypoints
      clipEigenVector(&waypoints_.positions, kNumWaypointsThreshold);
      clipEigenVector(&waypoints_.velocities, kNumWaypointsThreshold);
      clipEigenVector(&waypoints_.accelerations, kNumWaypointsThreshold);
      clipEigenVector(&waypoints_.jerks, kNumWaypointsThreshold);
      clipEigenVector(&waypoints_.elapsed_times, kNumWaypointsThreshold);
      // Eigen vectors do not have a "back" member function
      goal_joint_state_.position = waypoints_.positions[kNumWaypointsThreshold - 1];
      goal_joint_state_.velocity = waypoints_.velocities[kNumWaypointsThreshold - 1];
      goal_joint_state_.acceleration = waypoints_.accelerations[kNumWaypointsThreshold - 1];
      desired_duration_ = waypoints_.elapsed_times[kNumWaypointsThreshold - 1];
    }
  }

  // Normal mode: Error if we extended the duration to the maximum and it still wasn't successful
  if (!configuration_.use_streaming_mode &&
      index_last_successful_ < static_cast<size_t>(waypoints_.elapsed_times.size() - 1))
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

ErrorCodeEnum SingleJointGenerator::positionVectorLimitLookAhead(size_t* index_last_successful)
{
  ErrorCodeEnum error_code = forwardLimitCompensation(index_last_successful);
  if (error_code)
    return error_code;

  // Re-compile the position with these modifications.
  // Ensure the first and last elements are a perfect match with initial/final
  // conditions
  const long double one_sixth = 0.166667;
  // Initial waypoint
  waypoints_.positions(0) = current_joint_state_.position;
  for (size_t index = 1; index < static_cast<size_t>(waypoints_.positions.size()) - 1; ++index)
    waypoints_.positions(index) = waypoints_.positions(index - 1) +
                                  waypoints_.velocities(index - 1) * configuration_.timestep +
                                  0.5 * waypoints_.accelerations(index - 1) * pow(configuration_.timestep, 2) +
                                  one_sixth * waypoints_.jerks(index - 1) * pow(configuration_.timestep, 3);

  // Final waypoint should match the goal, unless trajectory was cut short for streaming mode
  if (!configuration_.use_streaming_mode)
    waypoints_.positions(waypoints_.positions.size() - 1) = goal_joint_state_.position;

  return error_code;
}

void SingleJointGenerator::calculateDerivativesFromPosition()
{
  // From position vector, approximate vel/accel/jerk.
  waypoints_.velocities = discreteDifferentiationWithFiltering(
      waypoints_.positions, configuration_.timestep, current_joint_state_.velocity, DEFAULT_FILTER_COEFFICIENT);
  calculateDerivativesFromVelocity();
}

void SingleJointGenerator::calculateDerivativesFromVelocity()
{
  // From velocity vector, approximate accel/jerk.
  waypoints_.accelerations = discreteDifferentiationWithFiltering(
      waypoints_.velocities, configuration_.timestep, current_joint_state_.acceleration, DEFAULT_FILTER_COEFFICIENT);
  waypoints_.jerks = discreteDifferentiationWithFiltering(waypoints_.accelerations, configuration_.timestep,
                                                          0.0 /*initial value*/, DEFAULT_FILTER_COEFFICIENT);
}

void SingleJointGenerator::updateTrajectoryDuration(const long double new_trajectory_duration)
{
  // The trajectory will be forced to have this duration (or fail) because
  // max_duration == desired_duration
  desired_duration_ = new_trajectory_duration;
  configuration_.max_duration = new_trajectory_duration;
}

void SingleJointGenerator::setInternalWaypointsData(const VectorXlong& positions, const VectorXlong& velocities,
                                                    const VectorXlong& accelerations, const VectorXlong& jerks,
                                                    const VectorXlong& elapsed_times)
{
  waypoints_.positions = positions;
  waypoints_.velocities = velocities;
  waypoints_.accelerations = accelerations;
  waypoints_.jerks = jerks;
  waypoints_.elapsed_times = elapsed_times;
}
}  // end namespace trackjoint
