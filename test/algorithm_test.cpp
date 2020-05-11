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
protected:
  void TearDown() override
  {
  }

};  // class AlgorithmTest

TEST_F(AlgorithmTest, BackwardLimitCompensationTest)
{
  // Test whether BackwardLimitCompensation can compensate for velocity overshoot at a particular timestep

  // TrackJoint parameters
  int num_dof = 1;
  double timestep = 0.01;
  double desired_duration = 10 * timestep;
  double max_duration = desired_duration;
  double position_tolerance = 1e-8;
  bool use_streaming_mode = false;

  std::vector<KinematicState> current_joint_states, goal_joint_states;

  KinematicState joint_state;
  joint_state.position = -1;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  current_joint_states.push_back(joint_state);

  joint_state.position = -0.995;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  goal_joint_states.push_back(joint_state);

  std::vector<Limits> limits;
  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 1;
  single_joint_limits.acceleration_limit = 100;
  single_joint_limits.jerk_limit = 1000;
  limits.push_back(single_joint_limits);

  std::vector<JointTrajectory> output_trajectories(num_dof);

  TrajectoryGenerator traj_gen(num_dof, timestep, desired_duration, max_duration, current_joint_states,
                               goal_joint_states, limits, position_tolerance, use_streaming_mode);
  // The difference between intended distance traveled and distance traveled after compensation
  double difference = traj_gen.runBackwardLimitCompensationTest();
  // We expect a difference very close to zero
  EXPECT_NEAR(difference, 0, 1e-6);
}
}  // namespace trackjoint

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  return result;
}