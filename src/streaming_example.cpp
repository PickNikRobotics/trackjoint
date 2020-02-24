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
  constexpr size_t kJoint = 0;
  constexpr double kTimestep = 0.001;
  constexpr double kMaxDuration = 100;
  constexpr bool kUseHighSpeedMode = true;
  constexpr double kPositionTolerance = 1e-6;
  constexpr double kMinDesiredDuration = kTimestep;
  // Between iterations, skip this many waypoints.
  // Take kNewSeedStateIndex from the previous trajectory to start the new trajectory.
  // Minimum is 1.
  constexpr std::size_t kNewSeedStateIndex = 10;

  std::vector<trackjoint::KinematicState> start_state(kNumDof);
  std::vector<trackjoint::KinematicState> goal_joint_states(kNumDof);
  start_state[kJoint].position = 0.9;
  goal_joint_states[kJoint].position = -0.9;

  std::vector<trackjoint::Limits> limits(kNumDof);
  limits[kJoint].velocity_limit = 2;
  limits[kJoint].acceleration_limit = 2;
  limits[kJoint].jerk_limit = 2;

  // This is a best-case estimate, assuming the robot is already at maximum velocity
  double desired_duration = fabs(start_state[0].position - goal_joint_states[0].position) / limits[0].velocity_limit;
  // But, don't ask for a duration that is shorter than one timestep
  desired_duration = std::max(desired_duration, kMinDesiredDuration);

  // Generate initial trajectory
  std::vector<trackjoint::JointTrajectory> output_trajectories(kNumDof);
  trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, desired_duration, kMaxDuration, start_state,
                                           goal_joint_states, limits, kPositionTolerance, kUseHighSpeedMode);
  trackjoint::ErrorCodeEnum error_code = traj_gen.GenerateTrajectories(&output_trajectories);
  if (error_code != trackjoint::ErrorCodeEnum::kNoError)
  {
    std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
    return -1;
  }
  std::cout << "Initial trajectory calculation:" << std::endl;

  PrintJointTrajectory(kJoint, output_trajectories, desired_duration);

  // Until a generated trajectory has only 2 waypoints
  while (desired_duration > kTimestep &&
         (std::size_t)output_trajectories.at(kJoint).positions.size() > kNewSeedStateIndex)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, desired_duration, kMaxDuration, start_state,
                                             goal_joint_states, limits, kPositionTolerance, kUseHighSpeedMode);
    error_code = traj_gen.GenerateTrajectories(&output_trajectories);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Run time (microseconds): " <<
      std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() << std::endl;

    if (error_code != trackjoint::ErrorCodeEnum::kNoError)
    {
      std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
      return -1;
    }

    // Print the synchronized trajectories
    PrintJointTrajectory(kJoint, output_trajectories, desired_duration);

    // Get a new seed state for next trajectory generation
    start_state[kJoint].position = output_trajectories.at(kJoint).positions[kNewSeedStateIndex];
    start_state[kJoint].velocity = output_trajectories.at(kJoint).velocities[kNewSeedStateIndex];
    start_state[kJoint].acceleration = output_trajectories.at(kJoint).accelerations[kNewSeedStateIndex];

    // Shorten the desired duration as we get closer to goal
    // This is a best-case estimate, assuming the robot is already at maximum velocity
    desired_duration =
        fabs(start_state[kJoint].position - goal_joint_states[kJoint].position) / limits[kJoint].velocity_limit;
    desired_duration = std::max(desired_duration, kTimestep);
  }

  std::cout << "Done!" << std::endl;

  return 0;
}
