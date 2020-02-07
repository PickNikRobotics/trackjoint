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

#include <trackjoint/error_codes.h>
#include <trackjoint/joint_trajectory.h>
#include <trackjoint/trajectory_generator.h>
#include <chrono>
#include <fstream>

int main(int argc, char** argv)
{
  constexpr size_t kNumDof = 1;
  constexpr double kTimestep = 0.004;
  constexpr double kDesiredDuration = 10;
  constexpr double kMaxDuration = 100;

  std::vector<trackjoint::KinematicState> start_state(1);
  std::vector<trackjoint::KinematicState> goal_joint_states(1);
  start_state[0].position = 0.9;
  goal_joint_states[0].position = -0.9;

  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 2;
  single_joint_limits.jerk_limit = 500;
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);

  // Generate initial trajectory
  std::vector<trackjoint::JointTrajectory> output_trajectories(kNumDof);
  trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, kDesiredDuration, kMaxDuration, start_state,
                                           goal_joint_states, limits);
  trackjoint::ErrorCodeEnum error_code = traj_gen.GenerateTrajectories(&output_trajectories);
  if (error_code != trackjoint::ErrorCodeEnum::kNoError)
  {
    std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
    return -1;
  }
  std::cout << output_trajectories[0].positions.size() << std::endl;

  // Until a generated trajectory has only 2 waypoints
  while (output_trajectories[0].positions.size() > 2)
  {
    trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, kDesiredDuration, kMaxDuration, start_state,
                                             goal_joint_states, limits);
    traj_gen.GenerateTrajectories(&output_trajectories);

    if (error_code != trackjoint::ErrorCodeEnum::kNoError)
    {
      std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
      return -1;
    }

    // Print the synchronized trajectories
    std::size_t joint = 0;
    std::cout << "==========" << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "==========" << std::endl;
    std::cout << "Joint " << joint << std::endl;
    std::cout << "==========" << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "==========" << std::endl;
    std::cout << "  Num waypts: " << output_trajectories.at(joint).positions.size() << std::endl;
    std::cout << "Elapsed time: " << output_trajectories.at(joint).elapsed_times[0]
              << "  Position: " << output_trajectories.at(joint).positions[0]
              << "  Velocity: " << output_trajectories.at(joint).velocities[0]
              << "  Acceleration: " << output_trajectories.at(joint).accelerations[0]
              << "  Jerk: " << output_trajectories.at(joint).jerks[0]
              << std::endl;

    // Use second waypoint (index 1) for the new seed state
    start_state[0].position = output_trajectories.at(joint).positions[1];
    start_state[0].velocity = output_trajectories.at(joint).velocities[1];
    start_state[0].acceleration = output_trajectories.at(joint).accelerations[1];
  }

  return 0;
}
