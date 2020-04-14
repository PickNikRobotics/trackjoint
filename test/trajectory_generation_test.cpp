/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2019, PickNik Consulting
 *  All rights reserved.
 *
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Adversarial testing of TrackJoint
*/

// C++
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <string>

// Testing
#include <gtest/gtest.h>
#include "test_utilities.h"

// Target testing library
#include <trackjoint/trajectory_generator.h>
#include "ros/package.h"
#include "ros/ros.h"

// Preparing data file handling
#include <fstream>
std::string REF_PATH = ros::package::getPath("trackjoint");
std::string BASE_FILEPATH = REF_PATH + "/test/data/tj_output_joint";

namespace trackjoint
{
class TrajectoryGenerationTest : public ::testing::Test
{
public:
  TrajectoryGenerationTest()
  {
    // Default test parameters for 3 joints
    trackjoint::KinematicState joint_state;
    joint_state.position = -1;
    joint_state.velocity = 0;
    joint_state.acceleration = 0;
    current_joint_states_.push_back(joint_state);
    current_joint_states_.push_back(joint_state);
    current_joint_states_.push_back(joint_state);

    joint_state.position = -0.995;
    joint_state.velocity = 0;
    joint_state.acceleration = 0;
    goal_joint_states_.push_back(joint_state);
    goal_joint_states_.push_back(joint_state);
    goal_joint_states_.push_back(joint_state);

    trackjoint::Limits single_joint_limits;
    single_joint_limits.velocity_limit = 20;
    single_joint_limits.acceleration_limit = 200;
    single_joint_limits.jerk_limit = 20000;
    limits_.push_back(single_joint_limits);
    limits_.push_back(single_joint_limits);
    limits_.push_back(single_joint_limits);
  }

protected:
  // Default test parameters for 3 joints
  double timestep_ = 0.01;
  double desired_duration_ = 1;
  int num_dof_ = 3;
  double max_duration_ = 10;
  std::vector<trackjoint::KinematicState> current_joint_states_, goal_joint_states_;
  std::vector<trackjoint::Limits> limits_;
  double position_tolerance_ = 1e-6;
  bool use_streaming_mode_ = false;
};  // class TrajectoryGenerationTest

TEST_F(TrajectoryGenerationTest, EasyDefaultTrajectory)
{
  // Use the class defaults. This trajectory is easy, does not require limit
  // compensation or trajectory extension

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration_, current_joint_states_,
                                           goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.generateTrajectories(&output_trajectories));

  // Position error
  const double position_tolerance = 1e-4;
  const double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states_, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  uint num_waypoint_tolerance = 1;
  uint expected_num_waypoints = 1 + desired_duration_ / timestep_;
  EXPECT_NEAR(uint(output_trajectories[0].positions.size()), expected_num_waypoints, num_waypoint_tolerance);
  // Timestep
  const double timestep_tolerance = 0.0005;
  EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep_,
              timestep_tolerance);
}

TEST_F(TrajectoryGenerationTest, OneTimestepDuration)
{
  // Request a duration of one timestep

  const double desired_duration = 1 * timestep_;
  const double max_duration = 1 * timestep_;

  std::vector<trackjoint::KinematicState> current_joint_states = current_joint_states_;
  trackjoint::KinematicState joint_state;
  joint_state.position = 0;
  joint_state.velocity = 0.1;
  joint_state.acceleration = 0;
  current_joint_states[0] = joint_state;
  current_joint_states[1] = joint_state;
  current_joint_states[2] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states = goal_joint_states_;
  joint_state.position = 0.001;
  joint_state.velocity = 0.1;
  joint_state.acceleration = 0;
  goal_joint_states[0] = joint_state;
  goal_joint_states[1] = joint_state;
  goal_joint_states[2] = joint_state;

  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 20;
  single_joint_limits.jerk_limit = 2000;
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration, max_duration, current_joint_states,
                                           goal_joint_states, limits, position_tolerance_, use_streaming_mode_);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  traj_gen.generateTrajectories(&output_trajectories);

  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.generateTrajectories(&output_trajectories));

  // Position error
  double position_tolerance = 1e-4;
  double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  const double duration_tolerance = 5e-3;
  size_t vector_length = output_trajectories[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories[0].elapsed_times(vector_length), desired_duration, duration_tolerance);
  // Timestep
  const double timestep_tolerance = 0.0005;
  EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep_,
              timestep_tolerance);
}

TEST_F(TrajectoryGenerationTest, RoughlyTwoTimestepDuration)
{
  // Request a duration of approximately two timesteps

  const double timestep = 0.01;
  const double desired_duration = 0.019;
  const double max_duration = desired_duration;

  trackjoint::KinematicState joint_state;
  std::vector<trackjoint::KinematicState> goal_joint_states = goal_joint_states_;
  joint_state.position = -0.998;
  goal_joint_states[0] = joint_state;
  goal_joint_states[1] = joint_state;
  goal_joint_states[2] = joint_state;

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep, desired_duration, max_duration, current_joint_states_,
                                           goal_joint_states, limits_, position_tolerance_, use_streaming_mode_);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  traj_gen.generateTrajectories(&output_trajectories);

  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.generateTrajectories(&output_trajectories));

  // Position error
  const double position_tolerance = 1e-4;
  const double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  const double duration_tolerance = 5e-4;
  size_t vector_length = output_trajectories[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories[0].elapsed_times(vector_length), desired_duration, duration_tolerance);
  // Number of waypoints
  EXPECT_EQ(output_trajectories[0].elapsed_times.size(), 3);
  // Timestep
  double timestep_tolerance = 0.003;
  EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep,
              timestep_tolerance);
}

TEST_F(TrajectoryGenerationTest, FourTimestepDuration)
{
  // Request a duration of just four timesteps

  const double timestep = 0.01;
  const double desired_duration = 4 * timestep;
  const double max_duration = desired_duration;

  trackjoint::KinematicState joint_state;
  std::vector<trackjoint::KinematicState> goal_joint_states = goal_joint_states_;
  joint_state.position = -0.998;
  goal_joint_states[0] = joint_state;
  goal_joint_states[1] = joint_state;
  goal_joint_states[2] = joint_state;

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep, desired_duration, max_duration, current_joint_states_,
                                           goal_joint_states, limits_, position_tolerance_, use_streaming_mode_);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  traj_gen.generateTrajectories(&output_trajectories);

  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.generateTrajectories(&output_trajectories));

  // Position error
  const double position_tolerance = 1e-4;
  const double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  const double duration_tolerance = 5e-3;
  size_t vector_length = output_trajectories[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories[0].elapsed_times(vector_length), desired_duration, duration_tolerance);
  // Timestep
  const double timestep_tolerance = 0.0005;
  EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep,
              timestep_tolerance);
}

TEST_F(TrajectoryGenerationTest, SixTimestepDuration)
{
  // Request a duration of six timesteps

  const double desired_duration = 6 * timestep_;
  const double max_duration = desired_duration;

  std::vector<trackjoint::KinematicState> current_joint_states = current_joint_states_;
  trackjoint::KinematicState joint_state;
  joint_state.position = 0;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  current_joint_states[0] = joint_state;
  current_joint_states[1] = joint_state;
  current_joint_states[2] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states = goal_joint_states_;
  joint_state.position = 0.0001;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  goal_joint_states[0] = joint_state;
  goal_joint_states[1] = joint_state;
  goal_joint_states[2] = joint_state;

  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 20;
  single_joint_limits.jerk_limit = 2000;
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration, max_duration, current_joint_states,
                                           goal_joint_states, limits, position_tolerance_, use_streaming_mode_);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  traj_gen.generateTrajectories(&output_trajectories);

  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.generateTrajectories(&output_trajectories));

  // Position error
  double position_tolerance = 1e-4;
  double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  const double duration_tolerance = 5e-3;
  size_t vector_length = output_trajectories[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories[0].elapsed_times(vector_length), desired_duration, duration_tolerance);
  // Timestep
  const double timestep_tolerance = 0.0005;
  EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep_,
              timestep_tolerance);
}

TEST_F(TrajectoryGenerationTest, VelAccelJerkLimit)
{
  // Velocity, acceleration and jerk limits are hit

  const double max_duration = 20;

  std::vector<trackjoint::KinematicState> current_joint_states = current_joint_states_;
  trackjoint::KinematicState joint_state;
  current_joint_states[0] = joint_state;
  current_joint_states[1] = joint_state;
  current_joint_states[2] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states = goal_joint_states_;
  goal_joint_states[0] = joint_state;
  goal_joint_states[1] = joint_state;
  goal_joint_states[2] = joint_state;

  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 0.001;
  single_joint_limits.acceleration_limit = 0.0005;
  single_joint_limits.jerk_limit = 0.001;
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration, current_joint_states,
                                           goal_joint_states, limits, position_tolerance_, use_streaming_mode_);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  traj_gen.generateTrajectories(&output_trajectories);

  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.generateTrajectories(&output_trajectories));

  VerifyVelAccelJerkLimits(output_trajectories, limits);

  // Position error
  double position_tolerance = 1e-4;
  double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  const double duration_tolerance = 5e-3;
  size_t vector_length = output_trajectories[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories[0].elapsed_times(vector_length), desired_duration_, duration_tolerance);
  // Timestep
  const double timestep_tolerance = 0.0005;
  EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep_,
              timestep_tolerance);
}

TEST_F(TrajectoryGenerationTest, NoisyStreamingCommand)
{
  // Incoming command is a noisy sine wave

  const double timestep = 0.1;
  const double desired_duration = timestep;
  const double max_duration = 10;
  const size_t num_waypoints = 500;

  std::default_random_engine random_generator;
  std::normal_distribution<double> random_distribution(2.0, 1.5);

  std::vector<trackjoint::KinematicState> current_joint_states = current_joint_states_;
  std::vector<trackjoint::KinematicState> goal_joint_states = goal_joint_states_;
  trackjoint::KinematicState joint_state;
  joint_state.position = 0;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  current_joint_states[0] = joint_state;
  current_joint_states[1] = joint_state;
  current_joint_states[2] = joint_state;
  goal_joint_states[0] = joint_state;
  goal_joint_states[1] = joint_state;
  goal_joint_states[2] = joint_state;

  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 15;
  single_joint_limits.jerk_limit = 200;
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);

  Eigen::VectorXd x_desired(num_waypoints);
  Eigen::VectorXd x_smoothed(num_waypoints);
  Eigen::VectorXd time_vector(num_waypoints);

  double time = 0;
  // Create Trajectory Generator object
  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep, desired_duration, max_duration, current_joint_states,
                                           goal_joint_states, limits, position_tolerance_, use_streaming_mode_);

  for (size_t waypoint = 0; waypoint < num_waypoints; ++waypoint)
  {
    time = waypoint * timestep;

    time_vector(waypoint) = time;

    joint_state.position = 0.1 * sin(time) + 0.05 * random_distribution(random_generator);

    goal_joint_states[0] = joint_state;
    goal_joint_states[1] = joint_state;
    goal_joint_states[2] = joint_state;

    x_desired(waypoint) = goal_joint_states[0].position;

    traj_gen.reset(timestep, desired_duration, max_duration, current_joint_states, goal_joint_states, limits,
                   position_tolerance_, use_streaming_mode_);
    std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
    traj_gen.generateTrajectories(&output_trajectories);

    VerifyVelAccelJerkLimits(output_trajectories, limits);
    // Timestep
    const double timestep_tolerance = 0.25 * timestep;
    EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep,
                timestep_tolerance);

    // Save the first waypoint in x_smoothed...
    x_smoothed(waypoint) = output_trajectories.at(0).positions(1);
    // ... and setting the next current position as the updated x_smoothed
    joint_state.position = x_smoothed(waypoint);

    current_joint_states[0] = joint_state;
    current_joint_states[1] = joint_state;
    current_joint_states[2] = joint_state;
  }
  EXPECT_EQ(x_desired.size(), x_smoothed.size());
  // Duration
  uint num_waypoint_tolerance = 1;
  uint expected_num_waypoints = num_waypoints;
  EXPECT_NEAR(uint(x_smoothed.size()), expected_num_waypoints, num_waypoint_tolerance);
}

TEST_F(TrajectoryGenerationTest, OscillatingUR5TrackJointCase)
{
  // This test comes from a MoveIt trajectory.
  // It was successful but there was a position oscillation.

  const int num_dof = 6;
  const double max_duration = 10;
  const double timestep = 0.0075;

  std::vector<trackjoint::Limits> limits(num_dof);
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 0.5;
  single_joint_limits.acceleration_limit = 2;
  single_joint_limits.jerk_limit = 100;
  limits[0] = single_joint_limits;
  limits[1] = single_joint_limits;
  limits[2] = single_joint_limits;
  limits[3] = single_joint_limits;
  limits[4] = single_joint_limits;
  limits[5] = single_joint_limits;

  ////////////////////////////////////////////////
  // Get TrackJoint initial states and goal states
  ////////////////////////////////////////////////

  // Vector of start states - for each waypoint, for each joint
  std::vector<std::vector<trackjoint::KinematicState>> trackjt_current_joint_states;
  // Vector of goal states - for each waypoint, for each joint
  std::vector<std::vector<trackjoint::KinematicState>> trackjt_goal_joint_states;
  // Vector of goal states - for each waypoint
  std::vector<double> trackjt_desired_durations;

  trackjoint::KinematicState joint_state;

  std::vector<std::vector<double>> moveit_des_positions;
  std::vector<std::vector<double>> moveit_des_velocities;
  std::vector<std::vector<double>> moveit_des_accelerations;
  std::vector<std::vector<double>> moveit_times_from_start;

  // Reading MoveIt experimental data from .txt files
  moveit_des_positions = LoadWaypointsFromFile(REF_PATH + "/test/data/30_percent_speed_oscillation/moveit_des_pos.txt");
  moveit_des_velocities =
      LoadWaypointsFromFile(REF_PATH + "/test/data/30_percent_speed_oscillation/moveit_des_vel.txt");
  moveit_des_accelerations =
      LoadWaypointsFromFile(REF_PATH + "/test/data/30_percent_speed_oscillation/moveit_des_acc.txt");
  moveit_times_from_start =
      LoadWaypointsFromFile(REF_PATH + "/test/data/30_percent_speed_oscillation/moveit_time_from_start.txt");

  // For each MoveIt waypoint
  for (std::size_t point = 0; point < moveit_times_from_start.size() - 1; ++point)
  {
    std::vector<trackjoint::KinematicState> current_joint_states;
    std::vector<trackjoint::KinematicState> goal_joint_states;

    // for each joint
    for (std::size_t joint = 0; joint < num_dof; ++joint)
    {
      // Save the start state of the robot
      joint_state.position = moveit_des_positions[point][joint];
      joint_state.velocity = moveit_des_velocities[point][joint];
      joint_state.acceleration = moveit_des_accelerations[point][joint];

      current_joint_states.push_back(joint_state);

      // Save the goal state of the robot
      joint_state.position = moveit_des_positions[point + 1][joint];
      joint_state.velocity = moveit_des_velocities[point + 1][joint];
      joint_state.acceleration = moveit_des_accelerations[point + 1][joint];

      goal_joint_states.push_back(joint_state);
    }

    trackjt_current_joint_states.push_back(current_joint_states);
    trackjt_goal_joint_states.push_back(goal_joint_states);
    trackjt_desired_durations.push_back(moveit_times_from_start[point + 1][0] - moveit_times_from_start[point][0]);
  }

  // Create trajectory generator object
  trackjoint::TrajectoryGenerator traj_gen(num_dof, timestep, trackjt_desired_durations[0], max_duration,
                                           trackjt_current_joint_states[0], trackjt_goal_joint_states[0], limits,
                                           position_tolerance_, use_streaming_mode_);

  // Step through the saved waypoints and smooth them with TrackJoint
  for (std::size_t point = 0; point < trackjt_desired_durations.size(); ++point)
  {
    traj_gen.reset(timestep, trackjt_desired_durations[point], max_duration, trackjt_current_joint_states[point],
                   trackjt_goal_joint_states[point], limits, position_tolerance_, use_streaming_mode_);
    std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof);

    ErrorCodeEnum error_code = traj_gen.generateTrajectories(&output_trajectories);

    // Saving Trackjoint output to .csv files for plotting
    traj_gen.saveTrajectoriesToFile(output_trajectories, BASE_FILEPATH, point != 0);

    EXPECT_EQ(ErrorCodeEnum::kNoError, error_code);
    // Timestep
    const double timestep_tolerance = 0.25 * timestep;
    EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep,
                timestep_tolerance);
  }
  EXPECT_EQ(trackjt_current_joint_states.at(0).size(), trackjt_goal_joint_states.at(0).size());
}

TEST_F(TrajectoryGenerationTest, SuddenChangeOfDirection)
{
  // Test a "corner" trajectory.
  // Velocity flips from (0.01,0,0) to (0,0,-0.01)

  const double desired_duration = 4 * timestep_;
  const double max_duration = 1;

  std::vector<trackjoint::KinematicState> current_joint_states = current_joint_states_;
  trackjoint::KinematicState joint_state;
  joint_state.position = 0;
  joint_state.velocity = 0.01;
  joint_state.acceleration = 0;
  current_joint_states[0] = joint_state;
  joint_state.velocity = 0;
  current_joint_states[1] = joint_state;
  current_joint_states[2] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states = goal_joint_states_;
  joint_state.position = 0.01;
  joint_state.velocity = 0;
  goal_joint_states[0] = joint_state;
  joint_state.position = 0;
  goal_joint_states[1] = joint_state;
  joint_state.position = -0.01;
  joint_state.velocity = -0.01;
  goal_joint_states[2] = joint_state;

  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 20;
  single_joint_limits.acceleration_limit = 200;
  single_joint_limits.jerk_limit = 20000;
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration, max_duration, current_joint_states,
                                           goal_joint_states, limits, position_tolerance_, use_streaming_mode_);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  traj_gen.generateTrajectories(&output_trajectories);

  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.generateTrajectories(&output_trajectories));

  // Position error
  double position_tolerance = 1e-4;
  double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  const double duration_tolerance = 5e-3;
  size_t vector_length = output_trajectories[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories[0].elapsed_times(vector_length), desired_duration, duration_tolerance);
  // Timestep
  const double timestep_tolerance = 0.0005;
  EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep_,
              timestep_tolerance);
}

TEST_F(TrajectoryGenerationTest, LimitCompensation)
{
  // These test parameters require limit compensation

  std::vector<trackjoint::KinematicState> current_joint_states = current_joint_states_;
  trackjoint::KinematicState joint_state;
  joint_state.position = -1;
  joint_state.velocity = -0.1;
  joint_state.acceleration = 0;
  current_joint_states[0] = joint_state;
  current_joint_states[1] = joint_state;
  current_joint_states[2] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states = goal_joint_states_;
  joint_state.position = 3;
  joint_state.velocity = 1.9;
  joint_state.acceleration = 0;
  goal_joint_states[0] = joint_state;
  goal_joint_states[1] = joint_state;
  goal_joint_states[2] = joint_state;

  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 1e4;
  single_joint_limits.jerk_limit = 1e6;
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);

  const double desired_duration = 2.5;
  const double max_duration = desired_duration;
  const double timestep = 0.001;

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep, desired_duration, max_duration, current_joint_states,
                                           goal_joint_states, limits, position_tolerance_, use_streaming_mode_);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.generateTrajectories(&output_trajectories));

  // Position error
  const double position_tolerance = 1e-4;
  const double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  uint num_waypoint_tolerance = 1;
  uint expected_num_waypoints = 1 + desired_duration / timestep;
  EXPECT_NEAR(uint(output_trajectories[0].positions.size()), expected_num_waypoints, num_waypoint_tolerance);
  // Timestep
  const double timestep_tolerance = 0.0005;
  EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep,
              timestep_tolerance);
}

TEST_F(TrajectoryGenerationTest, DurationExtension)
{
  // The third joint cannot reach in the desired time, so trajectories must be extended

  std::vector<trackjoint::KinematicState> current_joint_states = current_joint_states_;
  trackjoint::KinematicState joint_state;
  joint_state.position = -1;
  joint_state.velocity = -0.1;
  joint_state.acceleration = 0;
  current_joint_states[0] = joint_state;
  current_joint_states[1] = joint_state;
  current_joint_states[2] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states = goal_joint_states_;
  // No position change for the first two joints
  joint_state.position = -1;
  joint_state.velocity = 1.9;
  joint_state.acceleration = 0;
  goal_joint_states[0] = joint_state;
  goal_joint_states[1] = joint_state;
  // Big position change for the third joint
  joint_state.position = 0;
  goal_joint_states[2] = joint_state;

  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 1e2;
  single_joint_limits.jerk_limit = 1e4;
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);

  const double desired_duration = 0.1;
  const double max_duration = 5;
  const double timestep = 0.001;

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep, desired_duration, max_duration, current_joint_states,
                                           goal_joint_states, limits, position_tolerance_, use_streaming_mode_);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.generateTrajectories(&output_trajectories));

  // Position error
  const double position_tolerance = 1e-4;
  const double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  const double expected_duration = 1.05;
  const double duration_tolerance = 5e-3;
  size_t vector_length = output_trajectories[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories[0].elapsed_times(vector_length), expected_duration, duration_tolerance);
  // Timestep
  const double timestep_tolerance = 0.0005;
  EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep,
              timestep_tolerance);
}

TEST_F(TrajectoryGenerationTest, PositiveAndNegativeLimits)
{
  // This test encounters negative and positive velocity limits and negative
  // jerk limits

  std::vector<trackjoint::KinematicState> current_joint_states = current_joint_states_;
  trackjoint::KinematicState joint_state;
  joint_state.position = -1;
  joint_state.velocity = -0.2;
  joint_state.acceleration = 0;
  current_joint_states[0] = joint_state;
  joint_state.position = -1;
  joint_state.velocity = 0.1;
  current_joint_states[1] = joint_state;
  joint_state.position = 1;
  joint_state.velocity = 0.2;
  current_joint_states[2] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states = goal_joint_states_;
  joint_state.position = -0.9;
  joint_state.velocity = 0.1;
  goal_joint_states[0] = joint_state;
  joint_state.position = -0.9;
  joint_state.velocity = -0.1;
  goal_joint_states[1] = joint_state;
  joint_state.position = 0.9;
  joint_state.velocity = 0;
  goal_joint_states[2] = joint_state;

  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 0.21;
  single_joint_limits.acceleration_limit = 20;
  single_joint_limits.jerk_limit = 10;
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);

  const double timestep = 0.001;
  const double desired_duration = 1800 * timestep;
  const double max_duration = 1800 * timestep;

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep, desired_duration, max_duration, current_joint_states,
                                           goal_joint_states, limits, position_tolerance_, use_streaming_mode_);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.generateTrajectories(&output_trajectories));

  // Position error
  const double position_tolerance = 1e-4;
  const double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  uint num_waypoint_tolerance = 1;
  uint expected_num_waypoints = 1 + desired_duration / timestep;
  EXPECT_NEAR(uint(output_trajectories[0].positions.size()), expected_num_waypoints, num_waypoint_tolerance);
  // Timestep
  const double timestep_tolerance = 0.0005;
  EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep,
              timestep_tolerance);
}

TEST_F(TrajectoryGenerationTest, TimestepDidNotMatch)
{
  // This test comes from MoveIt. Originally, the final timestep did not match the desired timestep.

  std::vector<trackjoint::KinematicState> current_joint_states(1);
  trackjoint::KinematicState joint_state;
  joint_state.position = 0.00596041;
  joint_state.velocity = -0.176232;
  joint_state.acceleration = -3.06289;
  current_joint_states[0] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states(1);
  joint_state.position = -0.00121542;
  joint_state.velocity = -0.289615;
  joint_state.acceleration = -2.88021;
  goal_joint_states[0] = joint_state;

  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 3.15;
  single_joint_limits.acceleration_limit = 5;
  single_joint_limits.jerk_limit = 5000;
  std::vector<trackjoint::Limits> limits(1, single_joint_limits);

  const double timestep = 0.0075;
  const double desired_duration = 0.028322;
  const double max_duration = 10;
  const int num_dof = 1;

  trackjoint::TrajectoryGenerator traj_gen(num_dof, timestep, desired_duration, max_duration, current_joint_states,
                                           goal_joint_states, limits, position_tolerance_, use_streaming_mode_);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof);

  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.inputChecking(current_joint_states, goal_joint_states, limits, timestep));
  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.generateTrajectories(&output_trajectories));

  // Position error
  const double position_tolerance = 2e-3;
  const double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  const double timestep_tolerance = 0.0005;
  EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep,
              timestep_tolerance);
}

TEST_F(TrajectoryGenerationTest, CustomerStreaming)
{
  // A customer-requested streaming test.
  // For simplicity, only Joint 0 is updated
  // This is also a good test of trajectory synchronization in streaming mode.

  constexpr size_t num_dof = 3;
  constexpr std::size_t joint_to_update = 0;
  constexpr double timestep = 0.001;
  constexpr double max_duration = 100;
  constexpr bool use_streaming_mode = true;
  // Position tolerance for each waypoint
  constexpr double waypoint_position_tolerance = 1e-5;
  // Tolerances for the final waypoint
  constexpr double final_position_tolerance = 1e-5;
  constexpr double final_velocity_tolerance = 1e-3;
  constexpr double final_acceleration_tolerance = 1e-2;
  constexpr double min_desired_duration = 10 * timestep;
  // Between iterations, skip this many waypoints.
  // Take kNextWaypoint from the previous trajectory to start the new trajectory.
  // Minimum is 1.
  constexpr std::size_t next_waypoint = 1;

  std::vector<trackjoint::KinematicState> start_state(num_dof);
  std::vector<trackjoint::KinematicState> goal_joint_states(num_dof);
  start_state[0].position = 0.9;
  start_state[1].position = 0.4;
  start_state[2].position = -1.7;
  goal_joint_states[0].position = -0.9;
  goal_joint_states[1].position = -0.9;
  goal_joint_states[2].position = -0.9;

  trackjoint::Limits limits_per_joint;
  limits_per_joint.velocity_limit = 2;
  limits_per_joint.acceleration_limit = 2;
  limits_per_joint.jerk_limit = 2;
  std::vector<trackjoint::Limits> limits = { limits_per_joint, limits_per_joint, limits_per_joint };

  // This is a best-case estimate, assuming the robot is already at maximum velocity
  double desired_duration =
      fabs(start_state[joint_to_update].position - goal_joint_states[joint_to_update].position) /
      limits[joint_to_update].velocity_limit;
  // But, don't ask for a duration that is shorter than one timestep
  desired_duration = std::max(desired_duration, min_desired_duration);

  // Generate initial trajectory
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof);
  trackjoint::TrajectoryGenerator traj_gen(num_dof, timestep, desired_duration, max_duration, start_state,
                                           goal_joint_states, limits, waypoint_position_tolerance, use_streaming_mode);
  trackjoint::ErrorCodeEnum error_code = traj_gen.generateTrajectories(&output_trajectories);
  EXPECT_EQ(error_code, trackjoint::ErrorCodeEnum::kNoError);

  double position_error = std::numeric_limits<double>::max();
  double velocity_error = std::numeric_limits<double>::max();
  double acceleration_error = std::numeric_limits<double>::max();

  while (fabs(position_error) > final_position_tolerance || fabs(velocity_error) > final_velocity_tolerance ||
         fabs(acceleration_error) > final_acceleration_tolerance)
  {
    traj_gen.reset(timestep, desired_duration, max_duration, start_state, goal_joint_states, limits,
                   waypoint_position_tolerance, use_streaming_mode);
    error_code = traj_gen.generateTrajectories(&output_trajectories);
    EXPECT_EQ(error_code, trackjoint::ErrorCodeEnum::kNoError);
    // Timestep
    const double timestep_tolerance = 0.0005;
    EXPECT_NEAR(output_trajectories[0].elapsed_times[1] - output_trajectories[0].elapsed_times[0], timestep,
                timestep_tolerance);
    // All components should have the same number of waypoints
    EXPECT_EQ(output_trajectories[0].elapsed_times.size(), output_trajectories[1].elapsed_times.size());
    EXPECT_EQ(output_trajectories[0].elapsed_times.size(), output_trajectories[2].elapsed_times.size());
    // All components should have the same duration
    EXPECT_EQ(output_trajectories[0].elapsed_times[output_trajectories[0].elapsed_times.size() - 1],
              output_trajectories[1].elapsed_times[output_trajectories[0].elapsed_times.size() - 1]);
    EXPECT_EQ(output_trajectories[0].elapsed_times[output_trajectories[0].elapsed_times.size() - 1],
              output_trajectories[1].elapsed_times[output_trajectories[2].elapsed_times.size() - 1]);

    // Get a new seed state for next trajectory generation
    if ((std::size_t)output_trajectories.at(joint_to_update).positions.size() > next_waypoint)
    {
      start_state[joint_to_update].position = output_trajectories.at(joint_to_update).positions[next_waypoint];
      start_state[joint_to_update].velocity = output_trajectories.at(joint_to_update).velocities[next_waypoint];
      start_state[joint_to_update].acceleration =
          output_trajectories.at(joint_to_update).accelerations[next_waypoint];
    }

    position_error = start_state[joint_to_update].position - goal_joint_states.at(joint_to_update).position;
    velocity_error = start_state[joint_to_update].velocity - goal_joint_states.at(joint_to_update).velocity;
    acceleration_error =
        start_state[joint_to_update].acceleration - goal_joint_states.at(joint_to_update).acceleration;

    // Shorten the desired duration as we get closer to goal
    desired_duration -= timestep;
    // But, don't ask for a duration that is shorter than the minimum
    desired_duration = std::max(desired_duration, min_desired_duration);
  }

  // If the test gets here, it passed.
}

TEST_F(TrajectoryGenerationTest, StreamingTooFewTimesteps)
{
  // An error should be thrown if streaming mode is enabled with a desired duration < kMinNumTimesteps

  use_streaming_mode_ = true;
  desired_duration_ = 9 * timestep_;

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration_, current_joint_states_,
                                           goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  EXPECT_EQ(ErrorCodeEnum::kLessThanTenTimestepsForStreamingMode,
            traj_gen.inputChecking(current_joint_states_, goal_joint_states_, limits_, timestep_));
}
}  // namespace trackjoint

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  return result;
}
