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
  for (size_t index = 0; index < num_waypoints; ++index)
  {
    if (fabs(waypoints_.jerks(index)) > kLimits.angular_jerk_limit)
    {
      if (waypoints_.jerks(index) > kLimits.angular_jerk_limit)
      {
        delta_j = waypoints_.jerks(index) - kLimits.angular_jerk_limit;
        waypoints_.jerks(index) = kLimits.angular_jerk_limit;
      }
      else if (waypoints_.jerks(index) < -kLimits.angular_jerk_limit)
      {
        delta_j = waypoints_.jerks(index) + kLimits.angular_jerk_limit;
        waypoints_.jerks(index) = -kLimits.angular_jerk_limit;
      }
      waypoints_.accelerations(index) = waypoints_.accelerations(index) + delta_j * kTimestep;
      delta_v = 0.5 * delta_j * kTimestep * kTimestep;
      waypoints_.velocities(index) = waypoints_.velocities(index) + delta_v;
      // Try adjusting the veloicty in previous timesteps to compensate for this limit, if needed
      VelocityCompensation();
    }
  }

  // Compensate for acceleration limits at each timestep, starting near the beginning

  // Compensate for velocity limits at each timestep, starting near the beginning

  return index_last_successful;
}

void SingleJointGenerator::VelocityCompensation()
{
  ;
}

ErrorCodeEnum SingleJointGenerator::PredictTimeToReach() {
  return ErrorCodeEnum::kNoError;
}

}  // end namespace trackjoint
