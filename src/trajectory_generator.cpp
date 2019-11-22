/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <trackjoint/trajectory_generator.h>
#include <fstream>

namespace trackjoint
{
TrajectoryGenerator::TrajectoryGenerator(uint num_dof, double timestep, 
	double desired_duration, double max_duration,
	std::vector<KinematicState> &current_joint_states,
	std::vector<KinematicState> &goal_joint_states,
	std::vector<Limits> &limits)
{
  ///////////////////////
  // Input error checking
  ///////////////////////

  ////////////////////////////////////////////
  // Preallocate / initialize member variables
  ////////////////////////////////////////////
  num_dof_ = num_dof;
  desired_duration_ = desired_duration_;

  /////////////////////////////////////////////////////////////////
  // Upsample if num. waypoints would be short. Helps with accuracy
  /////////////////////////////////////////////////////////////////
  upsampled_timestep_ = timestep;

  ///////////////////////////////////////////////////
  // Initialize a trajectory generator for each joint
  ///////////////////////////////////////////////////
  for (size_t joint=0; joint<num_dof_; ++joint)
  {
    single_joint_generators_.push_back(SingleJointGenerator(
      desired_duration_,
      max_duration,
      current_joint_states[joint],
      goal_joint_states[joint],
      limits[joint]));
  }
}

void TrajectoryGenerator::SaveTrajectoriesToFile(std::vector<std::vector<TrajectoryWaypoint>> &output_trajectories, const std::string &base_filepath)
{
  std::ofstream output_file;
  std::string output_path;

  for (size_t joint = 0; joint < output_trajectories.size(); ++joint)
  {
    output_path = base_filepath + std::to_string(joint+1) + ".csv";
    output_file.open(output_path);
    for (size_t waypoint = 0; waypoint < output_trajectories.at(joint).size(); ++waypoint)
    {
      output_file << output_trajectories.at(joint).at(waypoint).elapsed_time << " " <<
      output_trajectories.at(joint).at(waypoint).state.position << " " <<
      output_trajectories.at(joint).at(waypoint).state.velocity << " " <<
      output_trajectories.at(joint).at(waypoint).state.acceleration << std::endl;
    }
    output_file.close();
    output_file.clear();
  }
}

void TrajectoryGenerator::GenerateTrajectories(std::vector<std::vector<TrajectoryWaypoint>> &output_trajectories)
{
  /////////////////////////////////////////
  // Generate individual joint trajectories
  /////////////////////////////////////////
  for (SingleJointGenerator traj_gen : single_joint_generators_)
  {
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
  output_trajectories.resize(num_dof_);
  const size_t num_waypoints = 10;
  for (size_t joint = 0; joint < output_trajectories.size(); ++joint)
  {
    output_trajectories.at(joint).resize(num_waypoints);
    for (size_t waypoint = 0; waypoint < output_trajectories.at(joint).size(); ++waypoint)
    {
      output_trajectories.at(joint).at(waypoint).state.position = 1.0 * waypoint;
      output_trajectories.at(joint).at(waypoint).state.velocity = 1.0 * waypoint;
      output_trajectories.at(joint).at(waypoint).state.acceleration = 1.0 * waypoint;
      output_trajectories.at(joint).at(waypoint).elapsed_time = 1.0 * waypoint;
    }
  }
}
}  // end namespace trackjoint
