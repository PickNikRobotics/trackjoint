/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: A minimal example of smoothing a trajectory for one joint.
*/

#include "trackjoint/error_codes.h"
#include "trackjoint/joint_trajectory.h"
#include "trackjoint/trajectory_generator.h"
#include <chrono>
#include <fstream>

int main(int argc, char** argv)
{
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

  const double position_tolerance = 1e-6;

  // Initialize main class
  trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, kDesiredDuration, kMaxDuration, current_joint_states,
                                           goal_joint_states, limits, position_tolerance);

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

  return 0;
}
