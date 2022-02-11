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
   Desc: Some 3-DOF examples.
*/

#include <trackjoint/error_codes.h>
#include <trackjoint/joint_trajectory.h>
#include <trackjoint/trajectory_generator.h>
#include <chrono>
#include <fstream>

int main(int argc, char** argv)
{
  constexpr int num_dof = 3;
  const long double timestep = 0.0039;
  constexpr long double max_duration = 30;
  // Streaming mode returns just a few waypoints but executes very quickly.
  constexpr bool use_streaming_mode = false;
  // Position tolerance for each waypoint
  constexpr long double waypoint_position_tolerance = 1e-4;
  const std::string output_path_base =
      "/home/" + std::string(getenv("USER")) + "/Downloads/trackjoint_data/output_joint";

  std::vector<trackjoint::KinematicState> current_joint_states(3);
  trackjoint::KinematicState joint_state;
  joint_state.position = 1.23984;
  joint_state.velocity = 1.45704;
  joint_state.acceleration = -0.0196446;
  current_joint_states[0] = joint_state;
  joint_state.position = -1.12637;
  joint_state.velocity = -0.774015;
  joint_state.acceleration = 0.0104357;
  current_joint_states[1] = joint_state;
  joint_state.position = -0.014012;
  joint_state.velocity = 0.583426;
  joint_state.acceleration = -0.00786606;
  current_joint_states[2] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states(3);
  joint_state.position = 1.4757;
  joint_state.velocity = 1.45675;
  joint_state.acceleration = 0.126129;
  goal_joint_states[0] = joint_state;
  joint_state.position = -0.545844;
  joint_state.velocity = 1.81361;
  joint_state.acceleration = 1.93357;
  goal_joint_states[1] = joint_state;
  joint_state.position = -0.447873;
  joint_state.velocity = -1.35293;
  joint_state.acceleration = -1.43058;
  goal_joint_states[2] = joint_state;

  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 3.15;
  single_joint_limits.acceleration_limit = 5;
  single_joint_limits.jerk_limit = 10000;
  std::vector<trackjoint::Limits> limits(3, single_joint_limits);

  // Estimate trajectory duration
  // This is the fastest possible trajectory execution time, assuming the robot starts at full velocity.
  long double desired_duration =
      fabs(goal_joint_states[1].position - current_joint_states[1].position) / single_joint_limits.velocity_limit;
  std::cout << "Desired duration: " << desired_duration << '\n';

  // Initialize main class
  trackjoint::TrajectoryGenerator traj_gen(num_dof, timestep, desired_duration, max_duration, current_joint_states,
                                           goal_joint_states, limits, waypoint_position_tolerance, use_streaming_mode);
  traj_gen.reset(timestep, desired_duration, max_duration, current_joint_states, goal_joint_states, limits,
                 waypoint_position_tolerance, use_streaming_mode);

  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof);

  trackjoint::ErrorCodeEnum error_code =
      traj_gen.inputChecking(current_joint_states, goal_joint_states, limits, timestep);

  // Input error handling - if an error is found, the trajectory is not
  // generated.
  if (error_code != trackjoint::ErrorCodeEnum::NO_ERROR)
  {
    std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << '\n';
    return -1;
  }

  // Measure runtime
  auto start = std::chrono::system_clock::now();
  error_code = traj_gen.generateTrajectories(&output_trajectories);
  auto end = std::chrono::system_clock::now();

  // Trajectory generation error handling
  if (error_code != trackjoint::ErrorCodeEnum::NO_ERROR)
  {
    std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << '\n';
    return -1;
  }

  std::chrono::duration<long double> elapsed_seconds = end - start;

  std::cout << "Runtime: " << elapsed_seconds.count() << '\n';
  std::cout << "Num waypoints: " << output_trajectories.at(0).positions.size() << '\n';
  std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << '\n';

  // Save the synchronized trajectories to .csv files
  traj_gen.saveTrajectoriesToFile(output_trajectories, output_path_base);

  // Print the synchronized trajectories
  for (size_t joint = 0; joint < output_trajectories.size(); ++joint)
  {
    std::cout << "==========" << '\n';
    std::cout << '\n';
    std::cout << '\n';
    std::cout << "==========" << '\n';
    std::cout << "Joint " << joint << '\n';
    std::cout << "==========" << '\n';
    std::cout << '\n';
    std::cout << '\n';
    std::cout << "==========" << '\n';
    for (size_t waypoint = 0; waypoint < static_cast<size_t>(output_trajectories.at(joint).positions.size()); ++waypoint)
    {
      std::cout << "Elapsed time: " << output_trajectories.at(joint).elapsed_times(waypoint)
                << "  Position: " << output_trajectories.at(joint).positions(waypoint)
                << "  Velocity: " << output_trajectories.at(joint).velocities(waypoint)
                << "  Acceleration: " << output_trajectories.at(joint).accelerations(waypoint)
                << "  Jerk: " << output_trajectories.at(joint).jerks(waypoint) << '\n';
    }
  }
  std::cout << "============================================" << '\n';

  return 0;
}
