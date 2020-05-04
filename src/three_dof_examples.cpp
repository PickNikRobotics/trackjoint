/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: Some 3-DOF examples.
*/

#include <trackjoint/error_codes.h>
#include <trackjoint/joint_trajectory.h>
#include <trackjoint/trajectory_generator.h>
#include <chrono>
#include <fstream>

int main(int argc, char** argv)
{
  constexpr int num_dof = 3;
  const double timestep = 0.001;
  double desired_duration = 0.028322;
  constexpr double max_duration = 10;
  // Streaming mode returns just a few waypoints but executes very quickly.
  constexpr bool use_streaming_mode = false;
  // Position tolerance for each waypoint
  constexpr double waypoint_position_tolerance = 1e-5;
  const std::string output_path_base = "/home/" + std::string(getenv("USER")) + "/trackjoint_data/output_joint";

  ////////////////////////////////////////////////
  // First example - small motions, come to a halt
  ////////////////////////////////////////////////
  std::cout << "EXAMPLE 1" << std::endl;
  std::vector<trackjoint::KinematicState> current_joint_states(3);
  trackjoint::KinematicState joint_state;
  joint_state.position = 0.00596041;
  joint_state.velocity = -0.176232;
  joint_state.acceleration = -3.06289;
  current_joint_states[0] = joint_state;
  joint_state.position = -0.596041;
  joint_state.velocity = 0.2;
  joint_state.acceleration = 2.39;
  current_joint_states[1] = joint_state;
  joint_state.position = -0.1;
  joint_state.velocity = 0.13;
  joint_state.acceleration = 0.2;
  current_joint_states[2] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states(3);
  joint_state.position = -0.00121542;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  goal_joint_states[0] = joint_state;
  joint_state.position = -0.57;
  goal_joint_states[1] = joint_state;
  joint_state.position = 0;
  goal_joint_states[2] = joint_state;

  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 3.15;
  single_joint_limits.acceleration_limit = 5;
  single_joint_limits.jerk_limit = 5000;
  std::vector<trackjoint::Limits> limits(3, single_joint_limits);

  // Initialize main class
  trackjoint::TrajectoryGenerator traj_gen(num_dof, timestep, desired_duration, max_duration, current_joint_states,
                                           goal_joint_states, limits, waypoint_position_tolerance, use_streaming_mode);

  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof);

  trackjoint::ErrorCodeEnum error_code =
      traj_gen.inputChecking(current_joint_states, goal_joint_states, limits, timestep);

  // Input error handling - if an error is found, the trajectory is not
  // generated.
  if (error_code != trackjoint::ErrorCodeEnum::kNoError)
  {
    std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
    return -1;
  }

  // Measure runtime
  auto start = std::chrono::system_clock::now();
  error_code = traj_gen.generateTrajectories(&output_trajectories);
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
  traj_gen.saveTrajectoriesToFile(output_trajectories, output_path_base);

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
  std::cout << "============================================" << std::endl;
/*
  /////////////////////////////////
  // Second example - large motions
  /////////////////////////////////
  std::cout << "EXAMPLE 2" << std::endl;
  desired_duration = 1;

  joint_state.position = 0.74;
  joint_state.velocity = 0.33;
  joint_state.acceleration = 0.15;
  current_joint_states[0] = joint_state;
  joint_state.position = -0.59;
  joint_state.velocity = 0.2;
  joint_state.acceleration = 2.39;
  current_joint_states[1] = joint_state;
  joint_state.position = -1.1;
  joint_state.velocity = 1.13;
  joint_state.acceleration = 0.9;
  current_joint_states[2] = joint_state;

  joint_state.position = 0.93;
  joint_state.velocity = 0.1;
  joint_state.acceleration = -0.2;
  goal_joint_states[0] = joint_state;
  joint_state.position = -0.86;
  joint_state.velocity = 0.05;
  joint_state.acceleration = -0.12;
  goal_joint_states[1] = joint_state;
  joint_state.position = -0.42;
  joint_state.velocity = 0.59;
  joint_state.acceleration = -0.23;
  goal_joint_states[2] = joint_state;

  // Create a new traj gen object with new parameters
  traj_gen.~TrajectoryGenerator();
  new (&traj_gen)
      trackjoint::TrajectoryGenerator(num_dof, timestep, desired_duration, max_duration, current_joint_states,
                                      goal_joint_states, limits, waypoint_position_tolerance, use_streaming_mode);

  error_code = traj_gen.inputChecking(current_joint_states, goal_joint_states, limits, timestep);

  // Input error handling - if an error is found, the trajectory is not
  // generated.
  if (error_code != trackjoint::ErrorCodeEnum::kNoError)
  {
    std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
    return -1;
  }

  // Measure runtime
  start = std::chrono::system_clock::now();
  error_code = traj_gen.generateTrajectories(&output_trajectories);
  end = std::chrono::system_clock::now();

  // Trajectory generation error handling
  if (error_code != trackjoint::ErrorCodeEnum::kNoError)
  {
    std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
    return -1;
  }

  elapsed_seconds = end - start;

  std::cout << "Runtime: " << elapsed_seconds.count() << std::endl;
  std::cout << "Num waypoints: " << output_trajectories.at(0).positions.size() << std::endl;
  std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;

  // Save the synchronized trajectories to .csv files
  traj_gen.saveTrajectoriesToFile(output_trajectories, output_path_base);

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
  std::cout << "============================================" << std::endl;
*/
  return 0;
}
