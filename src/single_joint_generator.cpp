/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

#include <trackjoint/single_joint_generator.h>
#include <trackjoint/utilities.h>

namespace trackjoint {

SingleJointGenerator::SingleJointGenerator(
    double timestep, double desired_duration, double max_duration,
    const trackjoint::KinematicState &current_joint_state,
    const trackjoint::KinematicState &goal_joint_state, const trackjoint::Limits &limits) :
kTimestep(timestep),
kDesiredDuration(desired_duration),
kMaxDuration(max_duration),
kCurrentJointState(current_joint_state),
kGoalJointState(goal_joint_state),
kLimits(limits)
{
  // Waypoint times
  times_ = Eigen::VectorXd::LinSpaced(int(kDesiredDuration / kTimestep), 0, kDesiredDuration);
}

ErrorCodeEnum SingleJointGenerator::GenerateTrajectory() {
  waypoints_.positions = Interpolate();

  // From position vector, approximate velocity and acceleration.
  // velocity = (difference between adjacent position elements) / delta_t
  // acceleration = (difference between adjacent velocity elements) / delta_t
  waypoints_.velocities = DiscreteDifferentiation(waypoints_.positions, kTimestep);
  waypoints_.accelerations = DiscreteDifferentiation(waypoints_.velocities, kTimestep);
  waypoints_.jerks = DiscreteDifferentiation(waypoints_.accelerations, kTimestep);

  LimitCompensation();

  // TODO(andyz): check for duration extension
  return PredictTimeToReach();
}

Eigen::VectorXd SingleJointGenerator::Interpolate()
{
  // See De Luca, "Trajectory Planning" pdf, slide 19
  // Interpolate a smooth trajectory from initial to final state while matching boundary conditions.

  // De Luca uses tao to represent a normalized time
  Eigen::VectorXd tao = times_ / kDesiredDuration;

  // TODO(andyz): vectorize this calculation
  Eigen::VectorXd interpolated_position(tao.size());
  for (size_t index=0; index<tao.size(); ++index)
  {
    interpolated_position(index) = pow((1. - tao(index)), 3) * (
      (kCurrentJointState.position + (3. * kCurrentJointState.position + kCurrentJointState.velocity * kDesiredDuration)) * tao(index) + 
      (kCurrentJointState.acceleration * pow(kDesiredDuration, 2) + 6. * kCurrentJointState.velocity + 12. * kCurrentJointState.position) * pow(tao(index), 2) / 2.) +
      pow(tao(index), 3) * (kGoalJointState.position + (3. * kGoalJointState.position - kGoalJointState.velocity * kDesiredDuration) * (1. - tao(index)) + 
      (kGoalJointState.acceleration * pow(kDesiredDuration, 2) - 6. * kGoalJointState.velocity + 12. * kGoalJointState.position) * pow((1. - tao(index)), 2) / 2.);
  }

  return interpolated_position;
}

size_t SingleJointGenerator::LimitCompensation()
{
  // for (size_t i=0; i<waypoints_.velocities.size(); ++i)
  //   std::cout << waypoints_.accelerations(i) << std::endl;

  // Start with the assumption that the entire trajectory can be completed
  size_t index_last_successful = waypoints_.positions.size();
  bool successful_compensation = false;

  // Preallocate
  double delta_j(0), delta_a(0), delta_v(0);
  size_t num_waypoints = waypoints_.positions.size();

  // Compensate for jerk limits at each timestep, starting near the beginning
  // Do not want to affect the velocity at the last timestep
  for (size_t index = 0; index < (num_waypoints-1); ++index)
  {
    if (fabs(waypoints_.jerks(index)) > kLimits.jerk_limit)
    {
      if (waypoints_.jerks(index) > kLimits.jerk_limit)
      {
        delta_j = waypoints_.jerks(index) - kLimits.jerk_limit;
        waypoints_.jerks(index) = kLimits.jerk_limit;
      }
      else if (waypoints_.jerks(index) < -kLimits.jerk_limit)
      {
        delta_j = waypoints_.jerks(index) + kLimits.jerk_limit;
        waypoints_.jerks(index) = -kLimits.jerk_limit;
      }
      waypoints_.accelerations(index) = waypoints_.accelerations(index) + delta_j * kTimestep;
      delta_v = 0.5 * delta_j * kTimestep * kTimestep;
      waypoints_.velocities(index) = waypoints_.velocities(index) + delta_v;
      // Try adjusting the veloicty in previous timesteps to compensate for this limit, if needed
      successful_compensation = VelocityCompensation(index, delta_v);
    }
  }

  // Compensate for acceleration limits at each timestep, starting near the beginning
  // Do not want to affect the velocity at the last timestep
  for (size_t index = 0; index < (num_waypoints-1); ++index)
  {
    if (fabs(waypoints_.accelerations(index)) > kLimits.acceleration_limit)
    {
      double temp_accel = 0;
      if (waypoints_.accelerations(index) > kLimits.acceleration_limit)
      {
        delta_a = kLimits.acceleration_limit - waypoints_.accelerations(index);
        temp_accel = kLimits.acceleration_limit;
      }
      else if (waypoints_.accelerations(index) < -kLimits.acceleration_limit)
      {
        delta_a = -kLimits.acceleration_limit - waypoints_.accelerations(index);
        temp_accel = -kLimits.acceleration_limit;
      }

      // Check jerk limit before applying the change
      if (!(fabs(waypoints_.jerks(index-1) + delta_a / kTimestep) < kLimits.jerk_limit))
      {
        waypoints_.accelerations(index) = temp_accel;
      }
      else
      {
        index_last_successful = RecordFailureTime(index, index_last_successful);
        break;
      }

      waypoints_.jerks(index) = 0;
      delta_v = delta_a * kTimestep;
      waypoints_.velocities(index) = waypoints_.velocities(index) + delta_v;
      // Try decreasing the velocity in previous timesteps to compensate for this limit, if needed
      successful_compensation = VelocityCompensation(index, delta_v);
      if (!successful_compensation)
      {
        index_last_successful = RecordFailureTime(index, index_last_successful);
      }
    }
  }

  // Compensate for velocity limits at each timestep, starting near the beginning
  // Do not want to affect the velocity at the last timestep
  for (size_t index = 0; index < (num_waypoints-1); ++index)
  {
    ;
  }

  return index_last_successful;
}

size_t SingleJointGenerator::RecordFailureTime(size_t current_index, size_t index_last_successful)
{
  // Record the index when compensation first failed
  if (current_index < index_last_successful)
    index_last_successful = current_index;

  return index_last_successful;
}

bool SingleJointGenerator::VelocityCompensation(size_t limited_index, double excess_velocity)
{
  bool successful_compensation = false;

  // Add a bit of velocity at step i to compensate for the limit at timestep i+1
  for (size_t index = limited_index; limited_index>0; --limited_index)
  {
    // if there is some room to increase the velocity at timestep i
    if (fabs(waypoints_.velocities(index)) < kLimits.velocity_limit)
    {
      // If the full change can be made in this timestep
      if ((excess_velocity > 0 && waypoints_.velocities(index) < kLimits.velocity_limit - excess_velocity) ||
        (excess_velocity < 0 && waypoints_.velocities(index) > -kLimits.velocity_limit - excess_velocity))
      {
        double new_velocity = waypoints_.velocities(index) + excess_velocity;
        // Accel and jerk, calculated from the previous waypoints
        double backward_accel = (new_velocity - waypoints_.velocities(index-1)) / kTimestep;
        double backward_jerk = (backward_accel - (waypoints_.velocities(index-1) - waypoints_.velocities(index-2)) / kTimestep) / kTimestep;
        // Accel and jerk, calculated from upcoming waypoints
        double forward_accel = (waypoints_.velocities(index+1) - new_velocity) / kTimestep;
        double forward_jerk = (forward_accel - (new_velocity - waypoints_.velocities(index-1)) / kTimestep) / kTimestep;

        // Calculate this new velocity if it would not violate accel/jerk limits
        if ((fabs(backward_jerk) < kLimits.jerk_limit) &&
            (fabs(backward_accel) < kLimits.acceleration_limit) &&
            (fabs(forward_jerk) < kLimits.jerk_limit) &&
            (fabs(forward_accel) < kLimits.acceleration_limit))
        {
          waypoints_.velocities(index) = new_velocity;
          successful_compensation = true;
          break;
        }
      }
      // else, can't make all of the correction in this timestep, so make as much of a change as possible
      else
      {
        // This is what accel and jerk would be if we set velocity(index) to the limit
        double new_velocity = std::signbit(excess_velocity) * kLimits.velocity_limit;
        // Accel and jerk, calculated from the previous waypoints
        double backward_accel = (new_velocity - waypoints_.velocities(index-1)) / kTimestep;
        double backward_jerk = (backward_accel - (waypoints_.velocities(index-1) - waypoints_.velocities(index-2)) / kTimestep) / kTimestep;
        // Accel and jerk, calculated from upcoming waypoints
        double forward_accel = (waypoints_.velocities(index+1) - new_velocity) / kTimestep;
        double forward_jerk = (forward_accel - (new_velocity - waypoints_.velocities(index-1)) / kTimestep) / kTimestep;

        // Apply these changes if all limits are satisfied (for the prior waypoint and the future waypoint)
        if ((fabs(backward_accel) < kLimits.acceleration_limit) &&
           (fabs(backward_jerk) < kLimits.jerk_limit) &&
           (fabs(forward_accel) < kLimits.acceleration_limit) &&
           (fabs(forward_jerk) < kLimits.jerk_limit))
        {
           double delta_v = new_velocity - waypoints_.velocities(index);
           waypoints_.velocities(index) = new_velocity;
           excess_velocity = excess_velocity - delta_v;
        }
      }
    }
    // Clip velocities at min/max due to small rounding errors
    waypoints_.velocities(index) = std::min(std::max(waypoints_.velocities(index), -kLimits.velocity_limit), kLimits.velocity_limit);
  }
  return successful_compensation;
}

ErrorCodeEnum SingleJointGenerator::PredictTimeToReach() {
  return ErrorCodeEnum::kNoError;
}

}  // end namespace trackjoint
