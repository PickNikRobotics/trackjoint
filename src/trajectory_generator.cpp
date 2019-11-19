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

namespace trackjoint
{
TrajectoryGenerator::TrajectoryGenerator(const uint num_dof, const double timestep, 
	const double desired_duration, const double max_duration,
	std::vector<KinematicState> &current_joint_states,
	std::vector<KinematicState> &goal_joint_states,
	std::vector<CartesianLimits> &limits, const double velocity_tolerance,
	const double acceleration_tolerance, const double jerk_tolerance)
{
  ///////////////////////
  // Input error checking
  ///////////////////////

  num_dof_ = num_dof;

  /////////////////////////////////////////////////////////////////
  // Upsample if num. waypoints would be short. Helps with accuracy
  /////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////
  // Initialize a trajectory generator for each joint
  ///////////////////////////////////////////////////
}

void TrajectoryGenerator::GenerateTrajectories(std::vector<std::vector<TrajectoryWaypoint>> &output_trajectories)
{
  /////////////////////////////////////////
  // Generate individual joint trajectories
  /////////////////////////////////////////

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

void KinematicState::print()
{
  std::cout << "Position:" << std::endl;
  std::cout << this->position << std::endl << std::endl;

  std::cout << "Velocity:" << std::endl;
  std::cout << this->velocity << std::endl << std::endl;

  std::cout << "Acceleration:" << std::endl;
  std::cout << this->acceleration << std::endl;
}
}  // end namespace trackjoint
