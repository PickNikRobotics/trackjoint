/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2020, PickNik Consulting
 *  All rights reserved.
 *
 *********************************************************************/

/* Author: Andy Zelenak
   Desc:   Unit testing of TrackJoint functions
*/

// C++
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <string>

// Testing
#include <gtest/gtest.h>

// Target testing library
#include <trackjoint/trajectory_generator.h>
#include <trackjoint/utilities.h>
#include "ros/package.h"
#include "ros/ros.h"

namespace trackjoint
{
class AlgorithmTest : public ::testing::Test
{
public:
  AlgorithmTest() : output_trajectories_(num_dof_)
  {
    KinematicState joint_state;
    joint_state.position = -1;
    joint_state.velocity = 0;
    joint_state.acceleration = 0;
    current_joint_states_.push_back(joint_state);

    joint_state.position = -0.995;
    joint_state.velocity = 0;
    joint_state.acceleration = 0;
    goal_joint_states_.push_back(joint_state);

    Limits single_joint_limits;
    single_joint_limits.velocity_limit = 1;
    single_joint_limits.acceleration_limit = 100;
    single_joint_limits.jerk_limit = 1000;
    limits_.push_back(single_joint_limits);
  }

protected:
  // Default test parameters for 3 joints
  double timestep_ = 0.01;
  double desired_duration_ = 10 * timestep_;
  int num_dof_ = 1;
  double max_duration_ = desired_duration_;
  std::vector<KinematicState> current_joint_states_, goal_joint_states_;
  std::vector<Limits> limits_;
  double position_tolerance_ = 1e-8;
  bool use_streaming_mode_ = false;
  bool write_output_ = true;
  std::vector<JointTrajectory> output_trajectories_;

  void runBackwardLimitCompensationTest(Eigen::VectorXd original_velocities, size_t limited_index,
                                        Eigen::VectorXd input_velocities)
  {
    EXPECT_EQ(original_velocities.size(), input_velocities.size());

    // TODO(1035): Ideally this TrajectoryGenerator would be a local fixture variable, but its constructor would then
    // access uninitialized vectors
    TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration_, current_joint_states_,
                                 goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);

    // Set up input velocities as generator's trajectory
    size_t num_steps = input_velocities.size();
    traj_gen.single_joint_generators_[0].waypoints_.velocities.resize(num_steps);
    traj_gen.single_joint_generators_[0].waypoints_.velocities = input_velocities;
    traj_gen.single_joint_generators_[0].waypoints_.positions.resize(num_steps);
    traj_gen.single_joint_generators_[0].waypoints_.accelerations.resize(num_steps);
    traj_gen.single_joint_generators_[0].waypoints_.jerks.resize(num_steps);

    // Have generator compensate for velocity limit applied at limited_index
    double delta_v = input_velocities[limited_index] - original_velocities[limited_index];
    double success = traj_gen.single_joint_generators_[0].backwardLimitCompensation(limited_index, -delta_v);
    EXPECT_TRUE(success);

    // Check that sum(original_velocities) == sum(compensated_velocities)
    // This is equivalent to testing that the correct distance was traveled
    EXPECT_NEAR(original_velocities.sum(), traj_gen.single_joint_generators_[0].waypoints_.velocities.sum(), 1e-6);
  }
};  // class AlgorithmTest

TEST_F(AlgorithmTest, PositiveVelocityLimitBackwardLimitCompensationTest)
{
  // Test that backwardLimitCompensation compensates for delta_velocity after a velocity > limit has been limited

  size_t num_steps = 21;
  // The intended velocity:
  Eigen::VectorXd original_velocities(num_steps);
  original_velocities << 0.997, 0.998, 0.999, 0.99987, 0.99988, 0.99989, 0.99990, 0.99991, 0.99992, 0.99993, 0.99994,
      0.99995, 0.99996, 0.99997, 0.99998, 0.99999, 1.0, 1.00001, 1.00001, 1.00001, 1.00001;

  // Limit element 17, which is over the velocity limit
  size_t limited_index = 17;

  // Element 17 has been set to the limit (1.0), backwardLimitCompensation should make up for this
  Eigen::VectorXd limited_velocities(num_steps);
  limited_velocities << 0.997, 0.998, 0.999, 0.99987, 0.99988, 0.99989, 0.99990, 0.99991, 0.99992, 0.99993, 0.99994,
      0.99995, 0.99996, 0.99997, 0.99998, 0.99999, 1.0, 1.0 /* changed */, 1.00001, 1.00001, 1.00001;

  runBackwardLimitCompensationTest(original_velocities, limited_index, limited_velocities);
}

TEST_F(AlgorithmTest, NegativeVelocityLimitBackwardLimitCompensationTest)
{
  // Test that backwardLimitCompensation compensates for delta_velocity after a velocity < -limit has been limited

  size_t num_steps = 21;
  // The intended velocity:
  Eigen::VectorXd original_velocities(num_steps);
  original_velocities << -0.997, -0.998, -0.999, -0.99987, -0.99988, -0.99989, -0.99990, -0.99991, -0.99992, -0.99993,
      -0.99994, -0.99995, -0.99996, -0.99997, -0.99998, -0.99999, -1.0, -1.00001, -1.00001, -1.00001, -1.00001;

  // Limit element 17, which is under the negative velocity limit
  size_t limited_index = 17;

  // Element 17 has been set to the limit (-1.0), backwardLimitCompensation should make up for this
  Eigen::VectorXd limited_velocities(num_steps);
  limited_velocities << -0.997, -0.998, -0.999, -0.99987, -0.99988, -0.99989, -0.99990, -0.99991, -0.99992, -0.99993,
      -0.99994, -0.99995, -0.99996, -0.99997, -0.99998, -0.99999, -1.0, -1.0 /* changed */, -1.00001, -1.00001,
      -1.00001;

  runBackwardLimitCompensationTest(original_velocities, limited_index, limited_velocities);
}
}  // namespace trackjoint

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  return result;
}
