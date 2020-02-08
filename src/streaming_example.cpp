/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: Constantly replan a new trajectory as the robot moves toward goal pose, until it reaches the goal.
*/

#include "trackjoint/error_codes.h"
#include "trackjoint/joint_trajectory.h"
#include "trackjoint/trajectory_generator.h"
#include "trackjoint/utilities.h"
#include <chrono>
#include <fstream>

int main(int argc, char** argv)
{
  constexpr size_t kNumDof = 1;
  constexpr double kTimestep = 0.001;
  constexpr double kMaxDuration = 100;
  constexpr double kPositionTolerance = 1e-6;
  constexpr double kMinDesiredDuration = kTimestep;
  // Between iterations, skip this many waypoints.
  // Take kNewSeedStateIndex from the previous trajectory to start the new trajectory.
  // Minimum is 1.
  constexpr std::size_t kNewSeedStateIndex = 10;

  std::vector<trackjoint::KinematicState> start_state(1);
  std::vector<trackjoint::KinematicState> goal_joint_states(1);
  start_state[0].position = 0.9;
  goal_joint_states[0].position = -0.9;

  std::vector<trackjoint::Limits> limits(1);
  limits[0].velocity_limit = 2;
  limits[0].acceleration_limit = 2;
  limits[0].jerk_limit = 2;

  // This is a best-case estimate, assuming the robot is already at maximum velocity
  double desired_duration = fabs(start_state[0].position - goal_joint_states[0].position) / limits[0].velocity_limit;
  // But, don't ask for a duration that is shorter than one timestep
  desired_duration = std::max(desired_duration - kNewSeedStateIndex * kTimestep, kMinDesiredDuration);

  // Generate initial trajectory
  std::vector<trackjoint::JointTrajectory> output_trajectories(kNumDof);
  trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, desired_duration, kMaxDuration, start_state,
                                           goal_joint_states, limits, kPositionTolerance);
  trackjoint::ErrorCodeEnum error_code = traj_gen.GenerateTrajectories(&output_trajectories);
  if (error_code != trackjoint::ErrorCodeEnum::kNoError)
  {
    std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
    return -1;
  }
  std::cout << "Initial trajectory calculation:" << std::endl;
  std::size_t joint = 0;
  PrintJointTrajectory(joint, output_trajectories, desired_duration);

  // Until a generated trajectory has only 2 waypoints
  while (desired_duration > kTimestep)
  {
    trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, desired_duration, kMaxDuration, start_state,
                                             goal_joint_states, limits, kPositionTolerance);
    error_code = traj_gen.GenerateTrajectories(&output_trajectories);

    if (error_code != trackjoint::ErrorCodeEnum::kNoError)
    {
      std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
      return -1;
    }

    // Print the synchronized trajectories
    joint = 0;
    PrintJointTrajectory(joint, output_trajectories, desired_duration);

    // Get a new seed state for next trajectory generation
    start_state[0].position = output_trajectories.at(joint).positions[kNewSeedStateIndex];
    start_state[0].velocity = output_trajectories.at(joint).velocities[kNewSeedStateIndex];
    start_state[0].acceleration = output_trajectories.at(joint).accelerations[kNewSeedStateIndex];

    // Shorten the desired duration as we get closer to goal
    // This is a best-case estimate, assuming the robot is already at maximum velocity
    desired_duration = fabs(start_state[0].position - goal_joint_states[0].position) / limits[0].velocity_limit;
    desired_duration = std::max(desired_duration, kTimestep);
  }

  std::cout << "Done!" << std::endl;

  return 0;
}
