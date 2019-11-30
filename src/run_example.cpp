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
#include <fstream>

int main(int argc, char** argv) {
  const int kNumDof = 3;
  const double kTimestep = 0.001;
  const double kDesiredDuration = 2.5;
  const double kMaxDuration = 5;
  const std::string kOutputPathBase =
      "../trackjoint_data/output_joint";

  std::vector<trackjoint::KinematicState> current_joint_states;
  trackjoint::KinematicState joint_state;
  joint_state.position = -1;
  joint_state.velocity = -0.1;
  joint_state.acceleration = 0;
  current_joint_states.push_back(joint_state);
  current_joint_states.push_back(joint_state);
  current_joint_states.push_back(joint_state);

  std::vector<trackjoint::KinematicState> goal_joint_states;
  // No position change for the first two joints
  joint_state.position = -1;
  joint_state.velocity = 1.9;
  joint_state.acceleration = 0;
  goal_joint_states.push_back(joint_state);
  goal_joint_states.push_back(joint_state);
  // Big position change for the third joint
  joint_state.position = 4;
  goal_joint_states.push_back(joint_state);

  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 1e4;
  single_joint_limits.jerk_limit = 1e6;
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);

  // Initialize main class
  trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, kDesiredDuration,
                                           kMaxDuration, current_joint_states,
                                           goal_joint_states, limits);

  std::vector<trackjoint::JointTrajectory> output_trajectories(kNumDof);
  trackjoint::ErrorCodeEnum error_code = traj_gen.GenerateTrajectories(&output_trajectories);
  std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;

  // Save the synchronized trajectories to .csv files
  // traj_gen.SaveTrajectoriesToFile(output_trajectories, kOutputPathBase);

  // Print the synchronized trajectories
  for (size_t joint = 0; joint < output_trajectories.size(); ++joint) {
    std::cout << "==========" << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "==========" << std::endl;
    std::cout << "Joint " << joint << std::endl;
    std::cout << "==========" << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "==========" << std::endl;
    for (size_t waypoint = 0; waypoint < output_trajectories.at(joint).positions.size();
         ++waypoint) {
      std::cout << "Elapsed time: "
                 << output_trajectories.at(joint).elapsed_times(waypoint)
                 << "  Position: " << output_trajectories.at(joint).positions(waypoint) << std::endl;
    }
  }

  return 0;
}
