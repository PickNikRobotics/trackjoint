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

/* Author: Andy Zelenak
   Desc: An example of smoothing a trajectory for several joints.
*/

#include <trackjoint/trajectory_generator.h>

int main(int argc, char** argv)
{
  const int NUM_DOF = 3;
  const double TIMESTEP = 0.01;
  const double DESIRED_DURATION = 1;
  const double MAX_DURATION = 3;
  const double VELOCITY_TOLERANCE = 1e-4;
  const double ACCELERATION_TOLERANCE = 1e-4;
  const double JERK_TOLERANCE = 1e-4;

  std::vector<trackjoint::KinematicState> current_joint_states;
  std::vector<trackjoint::KinematicState> goal_joint_states;
  std::vector<trackjoint::CartesianLimits> limits;

  // Initialize main class
  trackjoint::TrajectoryGenerator traj_gen(NUM_DOF, TIMESTEP,
    DESIRED_DURATION, MAX_DURATION, current_joint_states, goal_joint_states,
    limits, VELOCITY_TOLERANCE, ACCELERATION_TOLERANCE, JERK_TOLERANCE);

  std::vector<std::vector<trackjoint::TrajectoryWaypoint>> output_trajectories;
  traj_gen.GenerateTrajectories(output_trajectories);

  // Print the synchronized trajectories
  for (size_t joint = 0; joint < output_trajectories.size(); ++joint)
  {
    for (size_t waypoint = 0; waypoint < output_trajectories.at(joint).size(); ++waypoint)
    {
      std::cout << output_trajectories.at(joint).at(waypoint).elapsed_time << std::endl;
    }
  }

  return 0;
}
