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
#include <iostream>
#include <string>
#include <random>
#include <Eigen/Dense>

// Testing
#include <gtest/gtest.h>
#include "test_utilities.h"

// Target testing library
#include <trackjoint/trajectory_generator.h>

// To be reconsidered afterwards - GSN CHANGE
#include <fstream>
std::string base_filepath = "/home/guilesn/trackjoint_data/UR5-oscillating-experiment/tj_output_joint";

namespace trackjoint {
class TrajectoryGenerationTest : public ::testing::Test {
 public:
  TrajectoryGenerationTest() {
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
  std::vector<trackjoint::KinematicState> current_joint_states_,
      goal_joint_states_;
  std::vector<trackjoint::Limits> limits_;
};  // class TrajectoryGenerationTest

TEST_F(TrajectoryGenerationTest, OneTimestepDuration)
{
  // Request a duration of one timestep

  const double kDesiredDuration = 1 * timestep_;
  const double kMaxDuration = 1 * timestep_;

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

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep_, kDesiredDuration,
                                           kMaxDuration, current_joint_states,
                                           goal_joint_states, limits);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  traj_gen.GenerateTrajectories(&output_trajectories);

  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.GenerateTrajectories(&output_trajectories));

  // Position error
  double position_tolerance = 1e-4;
  double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  const double kExpectedDuration = kDesiredDuration;
  const double kDurationTolerance = 5e-3;
  size_t vector_length = output_trajectories[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories[0].elapsed_times(vector_length), kExpectedDuration, kDurationTolerance);
}

TEST_F(TrajectoryGenerationTest, FourTimestepDuration) {
  // Request a duration of just four timesteps

  const double kTimestep = 0.01;
  const double kDesiredDuration = 4 * kTimestep;
  const double kMaxDuration = kDesiredDuration;

  trackjoint::KinematicState joint_state;
  std::vector<trackjoint::KinematicState> goal_joint_states =
      goal_joint_states_;
  joint_state.position = -0.998;
  goal_joint_states[0] = joint_state;
  goal_joint_states[1] = joint_state;
  goal_joint_states[2] = joint_state;

  trackjoint::TrajectoryGenerator traj_gen(
      num_dof_, kTimestep, kDesiredDuration, kMaxDuration,
      current_joint_states_, goal_joint_states, limits_);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  traj_gen.GenerateTrajectories(&output_trajectories);

  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.GenerateTrajectories(&output_trajectories));

  // Position error
  const double kPositionTolerance = 1e-4;
  const double kPositionError = trackjoint::CalculatePositionAccuracy(
      goal_joint_states, output_trajectories);
  EXPECT_LT(kPositionError, kPositionTolerance);
  // Duration
  const double kExpectedDuration = kDesiredDuration;
  const double kDurationTolerance = 5e-3;
  size_t vector_length = output_trajectories[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories[0].elapsed_times(vector_length), kExpectedDuration, kDurationTolerance);
}

TEST_F(TrajectoryGenerationTest, SixTimestepDuration)
{
  // Request a duration of six timesteps

  const double kDesiredDuration = 6 * timestep_;
  const double kMaxDuration = kDesiredDuration;

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

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep_, kDesiredDuration,
                                           kMaxDuration, current_joint_states,
                                           goal_joint_states, limits);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  traj_gen.GenerateTrajectories(&output_trajectories);

  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.GenerateTrajectories(&output_trajectories));

  // Position error
  double position_tolerance = 1e-4;
  double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  const double kExpectedDuration = kDesiredDuration;
  const double kDurationTolerance = 5e-3;
  size_t vector_length = output_trajectories[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories[0].elapsed_times(vector_length), kExpectedDuration, kDurationTolerance);
}

TEST_F(TrajectoryGenerationTest, TestVelAccelJerkLimit)
{
  // Velocity, acceleration and jerk limits are hit

  const double kMaxDuration = 20;

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

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_,
                                           kMaxDuration, current_joint_states,
                                           goal_joint_states, limits);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  traj_gen.GenerateTrajectories(&output_trajectories);

  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.GenerateTrajectories(&output_trajectories));

  VerifyVelAccelJerkLimits(output_trajectories, limits);

  // Position error
  double position_tolerance = 1e-4;
  double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  const double kDurationTolerance = 5e-3;
  size_t vector_length = output_trajectories[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories[0].elapsed_times(vector_length), desired_duration_, kDurationTolerance);
}

TEST_F(TrajectoryGenerationTest, TestNoisyStreamingCommand)
{
  // Incoming command is a noisy sine wave

  const double kTimestep = 0.1;
  const double kDesiredDuration = kTimestep;
  const double kMaxDuration = 10;
  const size_t kNumWaypoints = 500;

  std::default_random_engine random_generator;
  std::normal_distribution<double> random_distribution(2.0,1.5);

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

  Eigen::VectorXd x_desired(kNumWaypoints);
  Eigen::VectorXd x_smoothed(kNumWaypoints);
  Eigen::VectorXd time_vector(kNumWaypoints);

  double time = 0;

  for(size_t waypoint = 0; waypoint < kNumWaypoints; ++waypoint){
    time = waypoint * kTimestep;

    time_vector(waypoint) = time;

    joint_state.position = 0.1 * sin(time) + 0.05 * random_distribution(random_generator);

    goal_joint_states[0] = joint_state;
    goal_joint_states[1] = joint_state;
    goal_joint_states[2] = joint_state;

    x_desired(waypoint) = goal_joint_states[0].position;

    trackjoint::TrajectoryGenerator traj_gen(num_dof_, kTimestep, kDesiredDuration,
                                          kMaxDuration, current_joint_states,
                                          goal_joint_states, limits);
    std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
    traj_gen.GenerateTrajectories(&output_trajectories);

    VerifyVelAccelJerkLimits(output_trajectories, limits);

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
  uint expected_num_waypoints = kNumWaypoints;
  EXPECT_NEAR(uint(x_smoothed.size()), expected_num_waypoints, num_waypoint_tolerance);
}

TEST_F(TrajectoryGenerationTest, TestOscillatingUR5TrackJointCase)
{
  const int kNumDof = 6;
  const double kMaxDuration = 10;
  const double kTimestep = 0.0075;

  std::vector<trackjoint::Limits> limits(kNumDof);
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

  trackjoint::ErrorCodeEnum error_code;

  // CREATE VARIABLES THAT WILL RECEIVE DATA FROM .TXT FILES
  std::vector<std::vector<double>> moveit_des_positions;
  std::vector<std::vector<double>> moveit_des_velocities;
  std::vector<std::vector<double>> moveit_des_accelerations;
  std::vector<std::vector<double>> moveit_times_from_start;

  // CALL FUNCTION THAT READS DATA FROM .TXT FILES BELOW
  moveit_des_positions = LoadWaypointsFromFile("/home/guilesn/trackjoint_ws/src/trackjoint/test/moveit_des_pos.txt");
  moveit_des_velocities = LoadWaypointsFromFile("/home/guilesn/trackjoint_ws/src/trackjoint/test/moveit_des_vel.txt");
  moveit_des_accelerations = LoadWaypointsFromFile("/home/guilesn/trackjoint_ws/src/trackjoint/test/moveit_des_acc.txt");
  moveit_times_from_start = LoadWaypointsFromFile("/home/guilesn/trackjoint_ws/src/trackjoint/test/moveit_time_from_start.txt");

  // For each MoveIt waypoint
  for (std::size_t point = 0; point < moveit_times_from_start.size() - 1; ++point)
  {
    std::vector<trackjoint::KinematicState> current_joint_states;
    std::vector<trackjoint::KinematicState> goal_joint_states;

    // for each joint
    for (std::size_t joint = 0; joint<kNumDof; ++joint)
    {
      // Save the start state of the robot
      joint_state.position = moveit_des_positions[point][joint];
      joint_state.velocity = moveit_des_velocities[point][joint];
      joint_state.acceleration = moveit_des_accelerations[point][joint];

      current_joint_states.push_back(joint_state);

      // Save the goal state of the robot
      joint_state.position = moveit_des_positions[point+1][joint];
      joint_state.velocity = moveit_des_velocities[point+1][joint];
      joint_state.acceleration = moveit_des_accelerations[point+1][joint];

      goal_joint_states.push_back(joint_state);
    }

    trackjt_current_joint_states.push_back(current_joint_states);
    trackjt_goal_joint_states.push_back(goal_joint_states);
    trackjt_desired_durations.push_back(moveit_times_from_start[point+1][0] - moveit_times_from_start[point][0]);
  }

  // Step through the saved waypoints and smooth them with TrackJoint
  for (std::size_t point=0; point<trackjt_desired_durations.size(); ++point)
  {
    trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, trackjt_desired_durations[point],
                                        kMaxDuration, trackjt_current_joint_states[point],
                                        trackjt_goal_joint_states[point], limits);
    std::vector<trackjoint::JointTrajectory> output_trajectories(kNumDof);

    traj_gen.GenerateTrajectories(&output_trajectories);

    traj_gen.SaveTrajectoriesToFile(output_trajectories, base_filepath);

    EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.GenerateTrajectories(&output_trajectories));
  }
    // debugging file writing
    size_t number_of_lines = 0;
    std::string line;
    std::ifstream input_file("/home/guilesn/trackjoint_data/UR5-oscillating-experiment/tj_output_joint3.txt");
    while (std::getline(input_file, line)) ++number_of_lines;
    std::cout << "Number of lines in text file: " << number_of_lines << std::endl;

  // EXPECT_EQ(x_desired.size(), x_smoothed.size());
  // // Duration
  // uint num_waypoint_tolerance = 1;
  // uint expected_num_waypoints = kNumWaypoints;
  // EXPECT_NEAR(uint(x_smoothed.size()), expected_num_waypoints, num_waypoint_tolerance);
  // EXPECT_EQ(true, true);
}

TEST_F(TrajectoryGenerationTest, SuddenChangeOfDirection)
{
  // Test a "corner" trajectory.
  // Velocity flips from (0.01,0,0) to (0,0,-0.01)

  const double kDesiredDuration = 4 * timestep_;
  const double kMaxDuration = 1;

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

  trackjoint::TrajectoryGenerator traj_gen(num_dof_, timestep_, kDesiredDuration,
                                           kMaxDuration, current_joint_states,
                                           goal_joint_states, limits);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  traj_gen.GenerateTrajectories(&output_trajectories);

  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.GenerateTrajectories(&output_trajectories));

  // Position error
  double position_tolerance = 1e-4;
  double position_error = trackjoint::CalculatePositionAccuracy(goal_joint_states, output_trajectories);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  const double kExpectedDuration = kDesiredDuration;
  const double kDurationTolerance = 5e-3;
  size_t vector_length = output_trajectories[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories[0].elapsed_times(vector_length), kExpectedDuration, kDurationTolerance);
}

TEST_F(TrajectoryGenerationTest, EasyDefaultTrajectory)
{
  // Use the class defaults. This trajectory is easy, does not require limit compensation or trajectory extension

  trackjoint::TrajectoryGenerator traj_gen(
      num_dof_, timestep_, desired_duration_, max_duration_,
      current_joint_states_, goal_joint_states_, limits_);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.GenerateTrajectories(&output_trajectories));

  // Position error
  const double kPositionTolerance = 1e-4;
  const double kPositionError = trackjoint::CalculatePositionAccuracy(
      goal_joint_states_, output_trajectories);
  EXPECT_LT(kPositionError, kPositionTolerance);
  // Duration
  uint num_waypoint_tolerance = 1;
  uint expected_num_waypoints = 1 + desired_duration_ / timestep_;
  EXPECT_NEAR(uint(output_trajectories[0].positions.size()), expected_num_waypoints, num_waypoint_tolerance);
}

TEST_F(TrajectoryGenerationTest, LimitCompensation) {
  // These test parameters require limit compensation

  std::vector<trackjoint::KinematicState> current_joint_states =
      current_joint_states_;
  trackjoint::KinematicState joint_state;
  joint_state.position = -1;
  joint_state.velocity = -0.1;
  joint_state.acceleration = 0;
  current_joint_states[0] = joint_state;
  current_joint_states[1] = joint_state;
  current_joint_states[2] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states =
      goal_joint_states_;
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

  const double kDesiredDuration = 2.5;
  const double kMaxDuration = kDesiredDuration;
  const double kTimestep = 0.001;

  trackjoint::TrajectoryGenerator traj_gen(
      num_dof_, kTimestep, kDesiredDuration, kMaxDuration, current_joint_states,
      goal_joint_states, limits);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.GenerateTrajectories(&output_trajectories));

  // Position error
  const double kPositionTolerance = 1e-4;
  const double kPositionError = trackjoint::CalculatePositionAccuracy(
      goal_joint_states, output_trajectories);
  EXPECT_LT(kPositionError, kPositionTolerance);
  // Duration
  uint num_waypoint_tolerance = 1;
  uint expected_num_waypoints = 1 + kDesiredDuration / kTimestep;
  EXPECT_NEAR(uint(output_trajectories[0].positions.size()), expected_num_waypoints, num_waypoint_tolerance);
}

TEST_F(TrajectoryGenerationTest, DurationExtension) {
  // The third joint cannot reach in the desired time, so trajectories must be
  // extended

  std::vector<trackjoint::KinematicState> current_joint_states =
      current_joint_states_;
  trackjoint::KinematicState joint_state;
  joint_state.position = -1;
  joint_state.velocity = -0.1;
  joint_state.acceleration = 0;
  current_joint_states[0] = joint_state;
  current_joint_states[1] = joint_state;
  current_joint_states[2] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states =
      goal_joint_states_;
  // No position change for the first two joints
  joint_state.position = -1;
  joint_state.velocity = 1.9;
  joint_state.acceleration = 0;
  goal_joint_states[0] = joint_state;
  goal_joint_states[1] = joint_state;
  // Big position change for the third joint
  joint_state.position = 4;
  goal_joint_states[2] = joint_state;

  std::vector<trackjoint::Limits> limits;
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 1e2;
  single_joint_limits.jerk_limit = 1e4;
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);
  limits.push_back(single_joint_limits);

  const double kDesiredDuration = 2.5;
  const double kMaxDuration = 5;
  const double kTimestep = 0.001;

  trackjoint::TrajectoryGenerator traj_gen(
      num_dof_, kTimestep, kDesiredDuration, kMaxDuration, current_joint_states,
      goal_joint_states, limits);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.GenerateTrajectories(&output_trajectories));

  // Position error
  const double kPositionTolerance = 1e-4;
  const double kPositionError = trackjoint::CalculatePositionAccuracy(
      goal_joint_states, output_trajectories);
  EXPECT_LT(kPositionError, kPositionTolerance);
  // Duration
  const double kExpectedDuration = 5.626;
  const double kDurationTolerance = 5e-3;
  size_t vector_length = output_trajectories[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories[0].elapsed_times(vector_length), kExpectedDuration, kDurationTolerance);
}

TEST_F(TrajectoryGenerationTest, PositiveAndNegativeLimits) {
  // This test encounters negative and positive velocity limits and negative jerk limits

  std::vector<trackjoint::KinematicState> current_joint_states =
      current_joint_states_;
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

  std::vector<trackjoint::KinematicState> goal_joint_states =
      goal_joint_states_;
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

  const double kTimestep = 0.001;
  const double kDesiredDuration = 1800 * kTimestep;
  const double kMaxDuration = 1800 * kTimestep;

  trackjoint::TrajectoryGenerator traj_gen(
      num_dof_, kTimestep, kDesiredDuration, kMaxDuration, current_joint_states,
      goal_joint_states, limits);
  std::vector<trackjoint::JointTrajectory> output_trajectories(num_dof_);
  EXPECT_EQ(ErrorCodeEnum::kNoError, traj_gen.GenerateTrajectories(&output_trajectories));

  // Position error
  const double kPositionTolerance = 1e-4;
  const double kPositionError = trackjoint::CalculatePositionAccuracy(
      goal_joint_states, output_trajectories);
  EXPECT_LT(kPositionError, kPositionTolerance);
  // Duration
  uint num_waypoint_tolerance = 1;
  uint expected_num_waypoints = 1 + kDesiredDuration / kTimestep;
  EXPECT_NEAR(uint(output_trajectories[0].positions.size()), expected_num_waypoints, num_waypoint_tolerance);
}
}  // namespace trackjoint

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  return result;
}
