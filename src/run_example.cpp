/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: An example of smoothing a trajectory for several joints.
*/

#include <trackjoint/trajectory_generator.h>
#include <fstream>

int main(int argc, char** argv)
{
  const int NUM_DOF = 3;
  const double TIMESTEP = 0.01;
  const double DESIRED_DURATION = 1;
  const double MAX_DURATION = 3;
  const std::string output_path_base = "/home/guilesn/trackjoint_ws/plots/output_joint";

  std::vector<trackjoint::KinematicState> current_joint_states;
  std::vector<trackjoint::KinematicState> goal_joint_states;
  std::vector<trackjoint::Limits> limits;

  // Initialize main class
  trackjoint::TrajectoryGenerator traj_gen(NUM_DOF, TIMESTEP,
    DESIRED_DURATION, MAX_DURATION, current_joint_states, goal_joint_states,
    limits);

  std::vector<std::vector<trackjoint::TrajectoryWaypoint>> output_trajectories;
  traj_gen.GenerateTrajectories(output_trajectories);

  // Save the synchronized trajectories to .csv files
  //traj_gen.SaveTrajectoriesToFile(output_trajectories, output_path_base);

  // Print the synchronized trajectories
  for (size_t joint = 0; joint < output_trajectories.size(); ++joint)
  {
    for (size_t waypoint = 0; waypoint < output_trajectories.at(joint).size(); ++waypoint)
    {
      output_trajectories.at(joint).at(waypoint).state.print();
      std::cout << "Elapsed time: " << output_trajectories.at(joint).at(waypoint).elapsed_time << std::endl;
    }
  }

  return 0;
}
