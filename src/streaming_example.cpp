/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: An example of smoothing a trajectory for three joints.
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
  constexpr double kPositionTolerance = 1e-6;

  std::vector<trackjoint::KinematicState> initial_joint_states(1);
  std::vector<trackjoint::KinematicState> goal_joint_states(1);
  initial_joint_states[0].position = 0.9;
  goal_joint_states[0].position = -0.9;

  // This input state will be updated as the robot moves toward goal
  std::vector<trackjoint::KinematicState> current_state_input = initial_joint_states;

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
  trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, kDesiredDuration, kMaxDuration, current_state_input,
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
    trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, kDesiredDuration, kMaxDuration, current_state_input,
                                             goal_joint_states, limits);
    traj_gen.GenerateTrajectories(&output_trajectories);

    // Print the synchronized trajectories
    for (size_t joint = 0; joint < output_trajectories.size(); ++joint)
    {
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
      std::cout << "Elapsed time: " << output_trajectories.at(joint).elapsed_times.back()
                << "  Position: " << output_trajectories.at(joint).positions.back()
                << "  Velocity: " << output_trajectories.at(joint).velocities.back()
                << "  Acceleration: " << output_trajectories.at(joint).accelerations(waypoint).back()
                << "  Jerk: " << output_trajectories.at(joint).jerks(waypoint).back()
                << std::endl;
      }
    }

    // New seed state
  }




/*
  const int kNumDof = 1;
  const double kTimestep = 0.0075;
  const double kDesiredDuration = 0.028322;
  const double kMaxDuration = 10;
  const std::string kOutputPathBase = "/home/" + std::string(getenv("USER")) + "/trackjoint_data/output_joint";

  std::vector<trackjoint::KinematicState> current_joint_states(1);
  trackjoint::KinematicState joint_state;
  joint_state.position = 0.00596041;
  joint_state.velocity = -0.176232;
  joint_state.acceleration = -3.06289;
  current_joint_states[0] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states(1);
  joint_state.position = -0.00121542;
  joint_state.velocity = -0.289615;
  joint_state.acceleration = -2.88021;
  goal_joint_states[0] = joint_state;

  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 3.15;
  single_joint_limits.acceleration_limit = 5;
  single_joint_limits.jerk_limit = 5000;
  std::vector<trackjoint::Limits> limits(1, single_joint_limits);

  // Initialize main class
  trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, kDesiredDuration, kMaxDuration, current_joint_states,
                                           goal_joint_states, limits);

  std::vector<trackjoint::JointTrajectory> output_trajectories(kNumDof);

  trackjoint::ErrorCodeEnum error_code =
      traj_gen.InputChecking(current_joint_states, goal_joint_states, limits, kTimestep);

  // Input error handling - if an error is found, the trajectory is not
  // generated.
  if (error_code != trackjoint::ErrorCodeEnum::kNoError)
  {
    std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
    return -1;
  }

  // Measure runtime
  auto start = std::chrono::system_clock::now();
  error_code = traj_gen.GenerateTrajectories(&output_trajectories);
  auto end = std::chrono::system_clock::now();

  // Trajectory generation error handling
  if (error_code != trackjoint::ErrorCodeEnum::kNoError)
  {
    std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
    return -1;
  }

  std::chrono::duration<double> elapsed_seconds = end - start;

  std::cout << "Runtime: " << elapsed_seconds.count() << std::endl;
  std::cout << "Num waypoints: " << output_trajectories.at(0).positions.size() << std::endl;
  std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;

  // Save the synchronized trajectories to .csv files
  traj_gen.SaveTrajectoriesToFile(output_trajectories, kOutputPathBase);

  // Print the synchronized trajectories
  for (size_t joint = 0; joint < output_trajectories.size(); ++joint)
  {
    std::cout << "==========" << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "==========" << std::endl;
    std::cout << "Joint " << joint << std::endl;
    std::cout << "==========" << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "==========" << std::endl;
    for (size_t waypoint = 0; waypoint < static_cast<size_t>(output_trajectories.at(joint).positions.size());
         ++waypoint)
    {
      std::cout << "Elapsed time: " << output_trajectories.at(joint).elapsed_times(waypoint)
                << "  Position: " << output_trajectories.at(joint).positions(waypoint)
                << "  Velocity: " << output_trajectories.at(joint).velocities(waypoint)
                << "  Acceleration: " << output_trajectories.at(joint).accelerations(waypoint)
                << "  Jerk: " << output_trajectories.at(joint).jerks(waypoint) << std::endl;
    }
  }
*/

  return 0;
}
