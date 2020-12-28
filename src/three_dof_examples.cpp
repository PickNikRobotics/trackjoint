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
  constexpr int num_dof = 6;
  const double timestep = 0.005;
  constexpr double max_duration = 1.5;
  // Position tolerance for each waypoint
  constexpr double waypoint_position_tolerance = 1e-4;
  const std::string output_path_base =
      "/home/" + std::string(getenv("USER")) + "/Downloads/trackjoint_data/";

  std::vector<trackjoint::KinematicState> current_joint_states(6);
  trackjoint::KinematicState joint_state;
  joint_state.position = 0.238288;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  current_joint_states[0] = joint_state;
  joint_state.position = 0.378499;
  current_joint_states[1] = joint_state;
  joint_state.position = -1.63914;
  current_joint_states[2] = joint_state;
  joint_state.position = 1.10303;
  current_joint_states[3] = joint_state;
  joint_state.position = 0.225048;
  current_joint_states[4] = joint_state;
  joint_state.position = -1.55303;
  current_joint_states[5] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states(6);
  joint_state.position = 0.654188;
  goal_joint_states[0] = joint_state;
  joint_state.position = 0.788694;
  goal_joint_states[1] = joint_state;
  joint_state.position = -0.971772;
  goal_joint_states[2] = joint_state;
  joint_state.position = 1.37333;
  goal_joint_states[3] = joint_state;
  joint_state.position = 0.640917;
  goal_joint_states[4] = joint_state;
  joint_state.position = -1.56715;
  goal_joint_states[5] = joint_state;

  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2.6;
  single_joint_limits.acceleration_limit = 16;
  single_joint_limits.jerk_limit = 34.6;
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  single_joint_limits.velocity_limit = 3;
  single_joint_limits.acceleration_limit = 20;
  single_joint_limits.jerk_limit = 41.4;
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);

  // Estimate trajectory duration
  // This is the fastest possible trajectory execution time, assuming the robot starts at full velocity.
  double desired_duration =
      fabs(goal_joint_states[1].position - current_joint_states[1].position) / single_joint_limits.velocity_limit;
  std::cout << "Desired duration: " << desired_duration << std::endl;

  // Initialize main class
  trackjoint::TrajectoryGenerator traj_gen(num_dof, timestep, desired_duration, max_duration, current_joint_states,
                                           goal_joint_states, limits, waypoint_position_tolerance);
  traj_gen.reset(timestep, desired_duration, max_duration, current_joint_states, goal_joint_states, limits,
                 waypoint_position_tolerance);

  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof);

  trackjoint::ErrorCodeEnum error_code =
      traj_gen.inputChecking(current_joint_states, goal_joint_states, limits, timestep);

  // Input error handling - if an error is found, the trajectory is not
  // generated.
  if (error_code != trackjoint::ErrorCodeEnum::NO_ERROR)
  {
    std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << std::endl;
    return -1;
  }

  // Measure runtime
  auto start = std::chrono::system_clock::now();
  error_code = traj_gen.generateTrajectories(&output_trajectories);
  auto end = std::chrono::system_clock::now();

  // Trajectory generation error handling
  if (error_code != trackjoint::ErrorCodeEnum::NO_ERROR)
  {
    std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << std::endl;
    return -1;
  }

  std::chrono::duration<double> elapsed_seconds = end - start;

  std::cout << "Runtime: " << elapsed_seconds.count() << std::endl;
  std::cout << "Num waypoints: " << output_trajectories.at(0).positions.size() << std::endl;
  std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << std::endl;

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

  return 0;
}
