// Copyright 2021 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Andy Zelenak
   Desc: Constantly replan a new trajectory as the robot moves toward goal pose, until it reaches the goal.
*/

#include "trackjoint/error_codes.h"
#include "trackjoint/joint_trajectory.h"
#include "trackjoint/trajectory_generator.h"
#include "trackjoint/utilities.h"
#include <chrono>
#include <fstream>

int main(int argc, char** argv)
{
  // This example is for just one degree of freedom
  constexpr size_t num_dof = 1;
  // For readability, save the joint index
  constexpr size_t joint = 0;
  // Waypoints will be spaced at 1 ms
  constexpr double timestep = 0.001;
  constexpr double max_duration = 100;
  // Streaming mode returns just a few waypoints but executes very quickly.
  // It returns from 2-kNumWaypointsThreshold waypoints, depending on how many waypoints can be calculated on a first
  // pass.
  // Waypoint[0] is the current state of the robot
  constexpr bool use_streaming_mode = true;
  // Position tolerance for each waypoint
  constexpr double waypoint_position_tolerance = 1e-5;
  // Loop until these tolerances are achieved
  constexpr double final_position_tolerance = 1e-4;
  constexpr double final_velocity_tolerance = 1e-1;
  constexpr double final_acceleration_tolerance = 1e-1;
  // For streaming mode, it is important to keep the desired duration >=10 timesteps.
  // Otherwise, an error will be thrown. This helps with accuracy
  constexpr double min_desired_duration = timestep;
  // Between TrackJoint iterations, move ahead this many waypoints along the trajectory.
  constexpr std::size_t next_waypoint = 1;

  // Define start state and goal states.
  // Position, velocity, and acceleration default to 0.
  std::vector<trackjoint::KinematicState> start_state(num_dof);
  std::vector<trackjoint::KinematicState> goal_joint_states(num_dof);
  start_state[joint].position = 0.9;
  goal_joint_states[joint].position = -0.9;

  // A vector of vel/accel/jerk limits for each DOF
  std::vector<trackjoint::Limits> limits(num_dof);
  limits[joint].velocity_limit = 2;
  limits[joint].acceleration_limit = 2;
  limits[joint].jerk_limit = 2;

  // This is a best-case estimate, assuming the robot is already at maximum velocity
  double desired_duration = fabs(start_state[0].position - goal_joint_states[0].position) / limits[0].velocity_limit;
  // But, don't ask for a duration that is shorter than the minimum
  desired_duration = std::max(desired_duration, min_desired_duration);

  // Create object for trajectory generation
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof);
  trackjoint::TrajectoryGenerator traj_gen(num_dof, timestep, desired_duration, max_duration, start_state,
                                           goal_joint_states, limits, waypoint_position_tolerance, use_streaming_mode);
  traj_gen.reset(timestep, desired_duration, max_duration, start_state, goal_joint_states, limits,
                 waypoint_position_tolerance, use_streaming_mode);

  // An example of optional input validation
  trackjoint::ErrorCodeEnum error_code = traj_gen.inputChecking(start_state, goal_joint_states, limits, timestep);
  if (error_code)
  {
    std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << '\n';
    return -1;
  }

  // Generate the initial trajectory
  error_code = traj_gen.generateTrajectories(&output_trajectories);
  if (error_code != trackjoint::ErrorCodeEnum::NO_ERROR)
  {
    std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << '\n';
    return -1;
  }
  std::cout << "Initial trajectory calculation:" << '\n';
  PrintJointTrajectory(joint, output_trajectories, desired_duration);

  // Update the start state with the next waypoint
  start_state[joint].position = output_trajectories.at(joint).positions[next_waypoint];
  start_state[joint].velocity = output_trajectories.at(joint).velocities[next_waypoint];
  start_state[joint].acceleration = output_trajectories.at(joint).accelerations[next_waypoint];

  // Loop while these errors exceed tolerances
  double position_error = std::numeric_limits<double>::max();
  double velocity_error = std::numeric_limits<double>::max();
  double acceleration_error = std::numeric_limits<double>::max();

  // Loop until the tolerances are satisfied
  while (fabs(position_error) > final_position_tolerance || fabs(velocity_error) > final_velocity_tolerance ||
         fabs(acceleration_error) > final_acceleration_tolerance)
  {
    // Optionally, time TrackJoint performance
    auto start_time = std::chrono::high_resolution_clock::now();

    // This reset() function is more computationally efficient when TrackJoint is called with a high frequency
    traj_gen.reset(timestep, desired_duration, max_duration, start_state, goal_joint_states, limits,
                   waypoint_position_tolerance, use_streaming_mode);
    error_code = traj_gen.generateTrajectories(&output_trajectories);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Run time (microseconds): "
              << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() << '\n';

    if (error_code != trackjoint::ErrorCodeEnum::NO_ERROR)
    {
      std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << '\n';
      return -1;
    }

    // Print the synchronized trajectories
    PrintJointTrajectory(joint, output_trajectories, desired_duration);

    // Move forward one waypoint for the next iteration
    if ((std::size_t)output_trajectories.at(joint).positions.size() > next_waypoint)
    {
      start_state[joint].position = output_trajectories.at(joint).positions[next_waypoint];
      start_state[joint].velocity = output_trajectories.at(joint).velocities[next_waypoint];
      start_state[joint].acceleration = output_trajectories.at(joint).accelerations[next_waypoint];
    }
    else
    {
      // This should never happen
      std::cout << "Index error!" << '\n';
      return 1;
    }

    // Calculate errors so tolerances can be checked
    position_error = start_state[joint].position - goal_joint_states.at(joint).position;
    velocity_error = start_state[joint].velocity - goal_joint_states.at(joint).velocity;
    acceleration_error = start_state[joint].acceleration - goal_joint_states.at(joint).acceleration;

    position_error = start_state[joint].position - goal_joint_states.at(joint).position;
    velocity_error = start_state[joint].velocity - goal_joint_states.at(joint).velocity;
    acceleration_error = start_state[joint].acceleration - goal_joint_states.at(joint).acceleration;

    // Shorten the desired duration as we get closer to goal
    desired_duration -= timestep;
    // But, don't ask for a duration that is shorter than the minimum
    desired_duration = std::max(desired_duration, min_desired_duration);
  }

  std::cout << "Done!" << '\n';

  return 0;
}
