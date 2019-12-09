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

// Testing
#include <gtest/gtest.h>
#include "test_utilities.h"

// Target testing library
#include <trackjoint/trajectory_generator.h>

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
  // Request a duration of just four timesteps. The duration will need to be extended.

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

TEST_F(TrajectoryGenerationTest, SuddenChangeOfDirection)
{
  // Test a "corner" trajectory.
  // Velocity flips from (0.01,0,0) to (0,0,-0.01)
  // Orientation flips 90* about Y
  // (This is known as a singularity for 1,2,3-Euler angle order, so it also tests that)

  const double kDesiredDuration = 4 * timestep_;
  const double kMaxDuration = 1;

  std::vector<trackjoint::KinematicState> current_joint_states = current_joint_states_;
  trackjoint::KinematicState joint_state;
  joint_state.position = 0;
  joint_state.velocity = 0.01;
  joint_state.acceleration = 0;
  current_joint_states[0] = joint_state;
  current_joint_states[1] = joint_state;
  current_joint_states[2] = joint_state;

  std::vector<trackjoint::KinematicState> goal_joint_states = goal_joint_states_;
  joint_state.position = 0.01;
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
}  // namespace trackjoint

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  return result;
}
