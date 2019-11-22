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
    // Default timestep
    upsampled_timestep_(timestep)
{
  ///////////////////////
  // Input error checking
  ///////////////////////

  /////////////////////////////////////////////////////////////////
  // Upsample if num. waypoints would be short. Helps with accuracy
  /////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////
  // Initialize a trajectory generator for each joint
  ///////////////////////////////////////////////////
  for (size_t joint = 0; joint < kNumDof; ++joint) {
    single_joint_generators_.push_back(SingleJointGenerator(
        desired_duration_, max_duration, current_joint_states[joint],
        goal_joint_states[joint], limits[joint]));
  }
}

void TrajectoryGenerator::SaveTrajectoriesToFile(
    const std::vector<std::vector<TrajectoryWaypoint>> &output_trajectories,
    const std::string &base_filepath) const {
  std::ofstream output_file;
  std::string output_path;

  for (size_t joint = 0; joint < output_trajectories.size(); ++joint) {
    output_path = base_filepath + std::to_string(joint + 1) + ".csv";
    output_file.open(output_path);
    for (size_t waypoint = 0; waypoint < output_trajectories.at(joint).size();
         ++waypoint) {
      output_file
          << output_trajectories.at(joint).at(waypoint).elapsed_time << " "
          << output_trajectories.at(joint).at(waypoint).state.position << " "
          << output_trajectories.at(joint).at(waypoint).state.velocity << " "
          << output_trajectories.at(joint).at(waypoint).state.acceleration
          << std::endl;
    }
    output_file.close();
    output_file.clear();
  }
}

std::vector<std::vector<TrajectoryWaypoint>> TrajectoryGenerator::GenerateTrajectories() {
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
  std::vector<std::vector<TrajectoryWaypoint>> output_trajectories(kNumDof);
  const size_t kNumWaypoints = 10;
  for (size_t joint = 0; joint < output_trajectories.size(); ++joint) {
    output_trajectories.at(joint).resize(kNumWaypoints);
    for (size_t waypoint = 0; waypoint < output_trajectories.at(joint).size();
         ++waypoint) {
      output_trajectories.at(joint).at(waypoint).state.position =
          1.0 * waypoint;
      output_trajectories.at(joint).at(waypoint).state.velocity =
          1.0 * waypoint;
      output_trajectories.at(joint).at(waypoint).state.acceleration =
          1.0 * waypoint;
      output_trajectories.at(joint).at(waypoint).elapsed_time = 1.0 * waypoint;
    }
  }
  return output_trajectories;
}
}  // end namespace trackjoint
