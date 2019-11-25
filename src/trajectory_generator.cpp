/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

#include <trackjoint/trajectory_generator.h>
#include <fstream>

namespace trackjoint {
TrajectoryGenerator::TrajectoryGenerator(
    uint num_dof, double timestep, double desired_duration, double max_duration,
    const std::vector<KinematicState> &current_joint_states,
    const std::vector<KinematicState> &goal_joint_states,
    const std::vector<Limits> &limits) :
    kNumDof(num_dof),
    desired_duration_(desired_duration),
    max_duration_(max_duration),
    // Default timestep
    upsampled_timestep_(timestep)
{
  // Input error checking
  InputChecking();

  // Upsample if num. waypoints would be short. Helps with accuracy

  // Initialize a trajectory generator for each joint
  for (size_t joint = 0; joint < kNumDof; ++joint) {
    single_joint_generators_.push_back(SingleJointGenerator(
        upsampled_timestep_, desired_duration_, max_duration_, current_joint_states[joint],
        goal_joint_states[joint], limits[joint], kMaxNumWaypoints));
  }
}

ErrorCodeEnum TrajectoryGenerator::InputChecking()
{
  ErrorCodeEnum error_code = ErrorCodeEnum::kNoError;

  if (desired_duration_ > kMaxNumWaypoints * upsampled_timestep_)
  {
    // Print a warning but do not exit
    std::cout << "Capping duration at 100 waypoints to maintain determinism." << std::endl;
    desired_duration_ = kMaxNumWaypoints * upsampled_timestep_;
  }

  if (max_duration_ > kMaxNumWaypoints * upsampled_timestep_)
  {
    // Print a warning but do not exit
    std::cout << "Capping duration at 100 waypoints to maintain determinism." << std::endl;
    max_duration_ = kMaxNumWaypoints * upsampled_timestep_;
  }
}

void TrajectoryGenerator::SaveTrajectoriesToFile(
    const std::vector<JointTrajectory> &output_trajectories,
    const std::string &base_filepath) const {

  std::ofstream output_file;
  std::string output_path;

  for (size_t joint = 0; joint < output_trajectories.size(); ++joint) {
    output_path = base_filepath + std::to_string(joint + 1) + ".csv";
    output_file.open(output_path);
    for (size_t waypoint = 0; waypoint < output_trajectories.at(joint).positions.size();
         ++waypoint) {
      output_file
          << output_trajectories.at(joint).elapsed_times(waypoint) << " "
          << output_trajectories.at(joint).positions(waypoint) << " "
          << output_trajectories.at(joint).velocities(waypoint) << " "
          << output_trajectories.at(joint).accelerations(waypoint)
          << std::endl;
    }
    output_file.close();
    output_file.clear();
  }
}

void TrajectoryGenerator::GenerateTrajectories(std::vector<JointTrajectory> *output_trajectories) {
  /////////////////////////////////////////
  // Generate individual joint trajectories
  /////////////////////////////////////////
  for (SingleJointGenerator traj_gen : single_joint_generators_) {
    traj_gen.GenerateTrajectory();
  }

  ////////////////////////////////////
  // Synchronize trajectory components
  ////////////////////////////////////

  /////////////////////////////////////////////////
  // Downsample, if needed, to the correct timestep
  /////////////////////////////////////////////////

  ///////////////////////
  // Final error checking
  ///////////////////////

  ///////////////////////////////////////
  // TODO(andyz): remove this sample data
  // Create sample data for plotting
  ///////////////////////////////////////

  // for (size_t dof = 0; dof < kNumDof; ++dof)
  // {
  //   output_trajectories[dof] = ;
  // }
  const size_t kNumWaypoints = 10;
  for (size_t joint = 0; joint < output_trajectories->size(); ++joint) {
    // Size the vectors
    output_trajectories->at(joint).positions.resize(kNumWaypoints);
    output_trajectories->at(joint).velocities.resize(kNumWaypoints);
    output_trajectories->at(joint).accelerations.resize(kNumWaypoints);
    output_trajectories->at(joint).elapsed_times.resize(kNumWaypoints);

    for (size_t waypoint = 0; waypoint < kNumWaypoints;
         ++waypoint) {
      output_trajectories->at(joint).positions(waypoint) = 1.0 * waypoint;
      output_trajectories->at(joint).velocities(waypoint) = 1.0 * waypoint;
      output_trajectories->at(joint).accelerations(waypoint) = 1.0 * waypoint;
      output_trajectories->at(joint).elapsed_times(waypoint) = 1.0 * waypoint;
    }
  }

  return;
}
}  // end namespace trackjoint
