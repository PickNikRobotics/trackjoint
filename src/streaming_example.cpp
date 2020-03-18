/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

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
  constexpr size_t kNumDof = 1;
  // For readability, save the joint index
  constexpr size_t kJoint = 0;
  // Waypoints will be spaced at 1 ms
  constexpr double kTimestep = 0.001;
  constexpr double kMaxDuration = 100;
  // High-speed mode returns just a few waypoints but executes very quickly.
  // It returns from 2-10 waypoints, depending on how many waypoints can be calculated on a first pass.
  // Waypoint[0] is the current state of the robot
  constexpr bool kUseHighSpeedMode = true;
  // Position tolerance for each waypoint
  constexpr double kWaypointPositionTolerance = 1e-5;
  // Loop until these tolerances are achieved
  constexpr double kFinalPositionTolerance = 1e-4;
  constexpr double kFinalVelocityTolerance = 1e-1;
  constexpr double kFinalAccelerationTolerance = 1e-1;
  // For high-speed mode, it is important to keep the desired duration >=10 timesteps.
  // Otherwise, an error will be thrown. This helps with accuracy
  constexpr double kMinDesiredDuration = 10 * kTimestep;
  // Between TrackJoint iterations, move ahead this many waypoints along the trajectory.
  constexpr std::size_t kNextWaypoint = 1;

  // Define start state and goal states.
  // Position, velocity, and acceleration default to 0.
  std::vector<trackjoint::KinematicState> start_state(kNumDof);
  std::vector<trackjoint::KinematicState> goal_joint_states(kNumDof);
  start_state[kJoint].position = 0.9;
  goal_joint_states[kJoint].position = -0.9;

  // A vector of vel/accel/jerk limits for each DOF
  std::vector<trackjoint::Limits> limits(kNumDof);
  limits[kJoint].velocity_limit = 2;
  limits[kJoint].acceleration_limit = 2;
  limits[kJoint].jerk_limit = 2;

  // This is a best-case estimate, assuming the robot is already at maximum velocity
  double desired_duration = fabs(start_state[0].position - goal_joint_states[0].position) / limits[0].velocity_limit;
  // But, don't ask for a duration that is shorter than the minimum
  desired_duration = std::max(desired_duration, kMinDesiredDuration);

  // Create object for trajectory generation
  std::vector<trackjoint::JointTrajectory> output_trajectories(kNumDof);
  trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, desired_duration, kMaxDuration, start_state,
                                           goal_joint_states, limits, kWaypointPositionTolerance, kUseHighSpeedMode);

  // An example of optional input validation
  trackjoint::ErrorCodeEnum error_code = traj_gen.InputChecking(start_state, goal_joint_states, limits, kTimestep);
  if (error_code)
  {
    std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
    return -1;
  }

  // Generate the initial trajectory
  error_code = traj_gen.GenerateTrajectories(&output_trajectories);
  if (error_code != trackjoint::ErrorCodeEnum::kNoError)
  {
    std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
    return -1;
  }
  std::cout << "Initial trajectory calculation:" << std::endl;
  PrintJointTrajectory(kJoint, output_trajectories, desired_duration);

  // Update the start state with the next waypoint
  start_state[kJoint].position = output_trajectories.at(kJoint).positions[kNextWaypoint];
  start_state[kJoint].velocity = output_trajectories.at(kJoint).velocities[kNextWaypoint];
  start_state[kJoint].acceleration = output_trajectories.at(kJoint).accelerations[kNextWaypoint];

  // Loop while these errors exceed tolerances
  double position_error = std::numeric_limits<double>::max();
  double velocity_error = std::numeric_limits<double>::max();
  double acceleration_error = std::numeric_limits<double>::max();

  // Loop until the tolerances are satisfied
  while (fabs(position_error) > kFinalPositionTolerance || fabs(velocity_error) > kFinalVelocityTolerance ||
         fabs(acceleration_error) > kFinalAccelerationTolerance)
  {
    // Optionally, time TrackJoint performance
    auto start_time = std::chrono::high_resolution_clock::now();

    // This Reset() function is more computationally efficient when TrackJoint is called with a high frequency
    traj_gen.Reset(kTimestep, desired_duration, kMaxDuration, start_state, goal_joint_states, limits,
                   kWaypointPositionTolerance, kUseHighSpeedMode);
    error_code = traj_gen.GenerateTrajectories(&output_trajectories);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Run time (microseconds): "
              << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() << std::endl;

    if (error_code != trackjoint::ErrorCodeEnum::kNoError)
    {
      std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code) << std::endl;
      return -1;
    }

    // Print the synchronized trajectories
    PrintJointTrajectory(kJoint, output_trajectories, desired_duration);

    // Move forward one waypoint for the next iteration
    if ((std::size_t)output_trajectories.at(kJoint).positions.size() > kNextWaypoint)
    {
      start_state[kJoint].position = output_trajectories.at(kJoint).positions[kNextWaypoint];
      start_state[kJoint].velocity = output_trajectories.at(kJoint).velocities[kNextWaypoint];
      start_state[kJoint].acceleration = output_trajectories.at(kJoint).accelerations[kNextWaypoint];
    }
    else
    {
      // This should never happen
      std::cout << "Index error!" << std::endl;
      return 1;
    }

    // Calculate errors so tolerances can be checked
    position_error = start_state[kJoint].position - goal_joint_states.at(kJoint).position;
    velocity_error = start_state[kJoint].velocity - goal_joint_states.at(kJoint).velocity;
    acceleration_error = start_state[kJoint].acceleration - goal_joint_states.at(kJoint).acceleration;

    position_error = start_state[kJoint].position - goal_joint_states.at(kJoint).position;
    velocity_error = start_state[kJoint].velocity - goal_joint_states.at(kJoint).velocity;
    acceleration_error = start_state[kJoint].acceleration - goal_joint_states.at(kJoint).acceleration;

    // Shorten the desired duration as we get closer to goal
    desired_duration -= kTimestep;
    // But, don't ask for a duration that is shorter than the minimum
    desired_duration = std::max(desired_duration, kMinDesiredDuration);
  }

  std::cout << "Done!" << std::endl;

  return 0;
}
