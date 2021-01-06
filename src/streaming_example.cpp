/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: 6-DOF example from Rapid Robotics.
*/

#include <trackjoint/error_codes.h>
#include <trackjoint/joint_trajectory.h>
#include <trackjoint/trajectory_generator.h>
#include <chrono>
#include <fstream>

// For data file parsing
#include <iostream>
#include <ros/package.h>
#include <sstream>

void parseTimeParameterizedJointWaypoints(const std::string& filepath, trackjoint::JointTrajectory& traj)
{
  std::cout << filepath << std::endl;
  std::ifstream data_file(filepath);
  if(!data_file.is_open())
  {
    throw std::runtime_error("Could not open data file");
  }

  std::string line;
  std::stringstream ss;
  double value;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;

  // Skip the header line
  std::getline(data_file, line);

  while(std::getline(data_file, line))
  {
    ss = std::stringstream(line);

    // Extract each column
    ss >> value;
    if(ss.peek() == ',')
      ss.ignore();
    positions.push_back(value);
    ss >> value;
    if(ss.peek() == ',')
      ss.ignore();
    velocities.push_back(value);
    // Skip the "Accel" column, will use the "Clipped Accel" column
    ss >> value;
    if(ss.peek() == ',')
      ss.ignore();
    ss >> value;
    if(ss.peek() == ',')
      ss.ignore();
    accelerations.push_back(value);
  }

  // Creating an Eigen::Vector from std::vector is kind of annoying
  traj.positions = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(positions.data(), positions.size());
  traj.velocities = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(velocities.data(), velocities.size());
  traj.accelerations = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(accelerations.data(), accelerations.size());
}

int main(int argc, char** argv)
{
  // Parse positions/velocities/accelerations from spreadsheet
  std::string data_path = ros::package::getPath("trackjoint") + "/example_data/";

  // Joint 1
  std::string filepath = data_path + "j1.csv";
  trackjoint::JointTrajectory j1_input_traj;
  parseTimeParameterizedJointWaypoints(filepath, j1_input_traj);
  // Joint 2
  filepath = data_path + "j2.csv";
  trackjoint::JointTrajectory j2_input_traj;
  parseTimeParameterizedJointWaypoints(filepath, j2_input_traj);
  // Joint 3
  filepath = data_path + "j3.csv";
  trackjoint::JointTrajectory j3_input_traj;
  parseTimeParameterizedJointWaypoints(filepath, j3_input_traj);
  // Joint 4
  filepath = data_path + "j4.csv";
  trackjoint::JointTrajectory j4_input_traj;
  parseTimeParameterizedJointWaypoints(filepath, j4_input_traj);
  // Joint 5
  filepath = data_path + "j5.csv";
  trackjoint::JointTrajectory j5_input_traj;
  parseTimeParameterizedJointWaypoints(filepath, j5_input_traj);
  // Joint 6
  filepath = data_path + "j6.csv";
  trackjoint::JointTrajectory j6_input_traj;
  parseTimeParameterizedJointWaypoints(filepath, j6_input_traj);

  const double timestep = 0.001;
  const double desired_duration = timestep;

  constexpr uint num_dof = 1;
  const double max_duration = 1000 * timestep;
  // Position tolerance for each waypoint
  constexpr double waypoint_position_tolerance = 1e-3;
  const std::string output_path_base =
      "/home/" + std::string(getenv("USER")) + "/Downloads/trackjoint_data/";
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof);

  // Kinematic limits
  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2.6;
  single_joint_limits.acceleration_limit = 16;
  single_joint_limits.jerk_limit = 34.6;
  limits.push_back(single_joint_limits);
  // limits.push_back(single_joint_limits);
  // limits.push_back(single_joint_limits);
  // single_joint_limits.velocity_limit = 3;
  // single_joint_limits.acceleration_limit = 20;
  // single_joint_limits.jerk_limit = 41.4;
  // limits.push_back(single_joint_limits);
  // limits.push_back(single_joint_limits);
  // limits.push_back(single_joint_limits);

  // Current state
  std::vector<trackjoint::KinematicState> current_joint_states(num_dof);
  trackjoint::KinematicState joint_state;

  // Goal state
  std::vector<trackjoint::KinematicState> goal_joint_states(num_dof);

  // Initialize main class
  trackjoint::TrajectoryGenerator traj_gen(num_dof, timestep, desired_duration, max_duration, current_joint_states,
                                           goal_joint_states, limits, waypoint_position_tolerance);

  // Iterate over every waypoint
  for (uint waypt = 0; waypt < j1_input_traj.positions.size() - 1; ++waypt)
  {
    joint_state.position = j1_input_traj.positions[waypt];
    joint_state.velocity = j1_input_traj.velocities[waypt];
    joint_state.acceleration = 0; //j1_input_traj.accelerations[waypt];
    current_joint_states[0] = joint_state;
    // joint_state.position = j2_input_traj.positions[waypt];
    // joint_state.velocity = j2_input_traj.velocities[waypt];
    // joint_state.acceleration = j2_input_traj.accelerations[waypt];
    // current_joint_states[1] = joint_state;
    // joint_state.position = j3_input_traj.positions[waypt];
    // joint_state.velocity = j3_input_traj.velocities[waypt];
    // joint_state.acceleration = j3_input_traj.accelerations[waypt];
    // current_joint_states[2] = joint_state;
    // joint_state.position = j4_input_traj.positions[waypt];
    // joint_state.velocity = j4_input_traj.velocities[waypt];
    // joint_state.acceleration = j4_input_traj.accelerations[waypt];
    // current_joint_states[3] = joint_state;
    // joint_state.position = j5_input_traj.positions[waypt];
    // joint_state.velocity = j5_input_traj.velocities[waypt];
    // joint_state.acceleration = j5_input_traj.accelerations[waypt];
    // current_joint_states[4] = joint_state;
    // joint_state.position = j6_input_traj.positions[waypt];
    // joint_state.velocity = j6_input_traj.velocities[waypt];
    // joint_state.acceleration = j6_input_traj.accelerations[waypt];
    // current_joint_states[5] = joint_state;

    joint_state.position = j1_input_traj.positions[waypt + 1];
    joint_state.velocity = j1_input_traj.velocities[waypt + 1];
    joint_state.acceleration = 0; //j1_input_traj.accelerations[waypt + 1];
    goal_joint_states[0] = joint_state;
    // joint_state.position = j2_input_traj.positions[waypt + 1];
    // joint_state.velocity = j2_input_traj.velocities[waypt + 1];
    // joint_state.acceleration = j2_input_traj.accelerations[waypt + 1];
    // goal_joint_states[1] = joint_state;
    // joint_state.position = j3_input_traj.positions[waypt + 1];
    // joint_state.velocity = j3_input_traj.velocities[waypt + 1];
    // joint_state.acceleration = j3_input_traj.accelerations[waypt + 1];
    // goal_joint_states[2] = joint_state;
    // joint_state.position = j4_input_traj.positions[waypt + 1];
    // joint_state.velocity = j4_input_traj.velocities[waypt + 1];
    // joint_state.acceleration = j4_input_traj.accelerations[waypt + 1];
    // goal_joint_states[3] = joint_state;
    // joint_state.position = j5_input_traj.positions[waypt + 1];
    // joint_state.velocity = j5_input_traj.velocities[waypt + 1];
    // joint_state.acceleration = j5_input_traj.accelerations[waypt + 1];
    // goal_joint_states[4] = joint_state;
    // joint_state.position = j6_input_traj.positions[waypt + 1];
    // joint_state.velocity = j6_input_traj.velocities[waypt + 1];
    // joint_state.acceleration = j6_input_traj.accelerations[waypt + 1];
    // goal_joint_states[5] = joint_state;

    traj_gen.reset(timestep, desired_duration, max_duration, current_joint_states, goal_joint_states, limits,
                   waypoint_position_tolerance);

    // Input error handling - if an error is found, the trajectory is not generated.
    trackjoint::ErrorCodeEnum error_code =
        traj_gen.inputChecking(current_joint_states, goal_joint_states, limits, timestep);
    if (error_code != trackjoint::ErrorCodeEnum::NO_ERROR)
    {
      std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << std::endl;
      return -1;
    }

    // Measure runtime
//    auto start = std::chrono::system_clock::now();
    error_code = traj_gen.generateTrajectories(&output_trajectories);
//    auto end = std::chrono::system_clock::now();
//  std::chrono::duration<double> elapsed_seconds = end - start;
//  std::cout << "Runtime: " << elapsed_seconds.count() << std::endl;

    // Trajectory generation error handling
    if (error_code != trackjoint::ErrorCodeEnum::NO_ERROR)
    {
      std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << std::endl;
      return -1;
    }

    // Save the synchronized trajectories to .csv files
    if (waypt == 0)
      traj_gen.saveTrajectoriesToFile(output_trajectories, output_path_base, false /* append_single_waypoint */);
    // After the first 2 waypoints, just save one at a time
    else
      traj_gen.saveTrajectoriesToFile(output_trajectories, output_path_base, true /* append_single_waypoint */);
  }

  return 0;
}
