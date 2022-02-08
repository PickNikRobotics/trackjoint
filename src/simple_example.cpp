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
   Desc: A minimal example of smoothing a trajectory for one joint.
*/

#include "trackjoint/error_codes.h"
#include "trackjoint/joint_trajectory.h"
#include "trackjoint/trajectory_generator.h"
#include <chrono>
#include <fstream>

int main(int argc, char** argv)
{
  // This example is for just one degree of freedom
  constexpr int num_dof = 1;
  // Timestep. Units don't matter as long as they're consistent
  constexpr double timestep = 0.001;
  // TrackJoint is allowed to extend the trajectory up to this duration, if a solution at kDesiredDuration can't be
  // found
  constexpr double max_duration = 5;
  // streaming mode returns just a few waypoints but executes very quickly. We won't use it here -- we'll calculate
  // the whole trajectory at once.
  constexpr bool use_streaming_mode = false;
  // Optional logging of TrackJoint output
  const std::string output_path_base =
      "/home/" + std::string(getenv("USER")) + "/Downloads/trackjoint_data/output_joint";

  std::vector<trackjoint::KinematicState> current_joint_states(1);
  trackjoint::KinematicState joint_state;
  joint_state.position = 1.16431;
  joint_state.velocity = 2.66465;
  joint_state.acceleration = 0;
  // This is the initial state of the joint
  current_joint_states[0] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states(1);
  joint_state.position = 1.40264;
  joint_state.velocity = 2.88556;
  joint_state.acceleration = 0.0615633;
  goal_joint_states[0] = joint_state;

  trackjoint::Limits single_joint_limits;
  // Typically, jerk limit >> acceleration limit > velocity limit
  single_joint_limits.velocity_limit = 3.15;
  single_joint_limits.acceleration_limit = 5;
  single_joint_limits.jerk_limit = 100;
  std::vector<trackjoint::Limits> limits(1, single_joint_limits);

  // Estimate trajectory duration
  // This is the fastest possible trajectory execution time, assuming the robot starts at full velocity.
  double desired_duration =
      fabs(goal_joint_states[0].position - current_joint_states[0].position) / single_joint_limits.velocity_limit;
  std::cout << "Desired duration: " << desired_duration << '\n';

  // This descibes how far TrackJoint can deviate from a smooth, interpolated polynomial.
  // It is used for calculations internally. It should be set to a smaller number than your task requires.
  const double position_tolerance = 0.0001;

  // Instantiate a trajectory generation object
  trackjoint::TrajectoryGenerator traj_gen(num_dof, timestep, desired_duration, max_duration, current_joint_states,
                                           goal_joint_states, limits, position_tolerance, use_streaming_mode);
  traj_gen.reset(timestep, desired_duration, max_duration, current_joint_states, goal_joint_states, limits,
                 position_tolerance, use_streaming_mode);
  // This vector holds the trajectories for each DOF
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof);
  // Optionally, check user input for common errors, like current velocities being less than velocity limits
  trackjoint::ErrorCodeEnum error_code =
      traj_gen.inputChecking(current_joint_states, goal_joint_states, limits, timestep);

  // Input error handling - if an error is found, the trajectory is not generated.
  if (error_code != trackjoint::ErrorCodeEnum::NO_ERROR)
  {
    std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << '\n';
    return -1;
  }

  // Optionally, measure runtime
  auto start = std::chrono::system_clock::now();
  // This is where the magic happens
  error_code = traj_gen.generateTrajectories(&output_trajectories);
  auto end = std::chrono::system_clock::now();

  // Trajectory generation error handling
  if (error_code != trackjoint::ErrorCodeEnum::NO_ERROR)
  {
    std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << '\n';
    return -1;
  }

  std::chrono::duration<double> elapsed_seconds = end - start;

  std::cout << "Runtime: " << elapsed_seconds.count() << '\n';
  std::cout << "Num waypoints: " << output_trajectories.at(0).positions.size() << '\n';
  std::cout << "Error code: " << trackjoint::ERROR_CODE_MAP.at(error_code) << '\n';

  // Save the synchronized trajectories to .csv files
  traj_gen.saveTrajectoriesToFile(output_trajectories, output_path_base);

  // Print the synchronized trajectories.
  // Note: waypoint[0] should match the user-supplied start state
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

  return 0;
}
