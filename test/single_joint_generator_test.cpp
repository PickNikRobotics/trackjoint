/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2020, PickNik Consulting
 *  All rights reserved.
 *
 *********************************************************************/

/* Author: Dave Coleman, John Morris
   Desc:   Run TrackJoint single joint trajectory algorithm test cases
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
#include <trackjoint/single_joint_generator.h>
#include <trackjoint/utilities.h>
#include "ros/package.h"
#include "ros/ros.h"

// Preparing data file handling
#include <fstream>
std::string REF_PATH = ros::package::getPath("trackjoint");
std::string BASE_FILEPATH = REF_PATH + "/test/data/single_joint_output";

namespace trackjoint
{
class SingleJointGeneratorTest : public ::testing::Test
{
public:
  SingleJointGeneratorTest()
  {
    // Default test parameters
    current_joint_state_.position = 0;
    current_joint_state_.velocity = 0;
    current_joint_state_.acceleration = 0;
    goal_joint_state_.position = 1;
    goal_joint_state_.velocity = 0;
    goal_joint_state_.acceleration = 0;
    joint_limits_.velocity_limit = 20;
    joint_limits_.acceleration_limit = 200;
    joint_limits_.jerk_limit = 20000;
  }

protected:
  // Default test parameters
  double timestep_ = 0.01;
  double desired_duration_ = 1;
  Eigen::Index num_waypoints_ = 0;
  double max_duration_ = 0;
  KinematicState current_joint_state_, goal_joint_state_;
  Limits joint_limits_;
  double position_error_;
  double position_tolerance_ = 1e-4;
  bool use_streaming_mode_ = false;
  bool write_output_ = true;
  JointTrajectory output_trajectory_;
  // From trajectory_generator.h
  const size_t kNumWaypointsThreshold_ = 10;
  const size_t kMaxNumWaypointsFullTrajectory_ = 10000;

  Eigen::Index size()
  {
    assert (num_waypoints_ > 0);
    return num_waypoints_;
  }

  std::string name()
  {
    return ::testing::UnitTest::GetInstance()->current_test_info()->name();
  }

  void genTrajectory()
  {
    if (num_waypoints_ == 0)
      num_waypoints_ = 1 + desired_duration_ / timestep_;
    if (max_duration_ == 0)
      max_duration_ = desired_duration_;

    SingleJointGenerator gen(kNumWaypointsThreshold_,
                             kMaxNumWaypointsFullTrajectory_);
    gen.reset(timestep_, max_duration_, current_joint_state_,
              goal_joint_state_, joint_limits_, num_waypoints_,
              position_tolerance_, use_streaming_mode_, true);
    int err = gen.generateTrajectory();
    std::cerr << name() << " Error: "
              << trackjoint::ERROR_CODE_MAP.at(err) << std::endl;
    EXPECT_EQ(ErrorCodeEnum::NO_ERROR, err);
    output_trajectory_ = gen.getTrajectory();
  }

  void checkBounds()
  {
      if (output_trajectory_.elapsed_times.size() == 0)
        return;

      // Sanity check vector lengths
      EXPECT_EQ(output_trajectory_.elapsed_times.size(), size());
      EXPECT_EQ(output_trajectory_.positions.size(), size());
      EXPECT_EQ(output_trajectory_.velocities.size(), size());
      EXPECT_EQ(output_trajectory_.accelerations.size(), size());
      EXPECT_EQ(output_trajectory_.jerks.size(), size());

      double elapsed_time = output_trajectory_.elapsed_times[size() - 1];

      // Get estimate of min/max position and velocity using start and
      // end states
      double min_pos = std::min(current_joint_state_.position,
                                goal_joint_state_.position);
      double max_pos = std::max(current_joint_state_.position,
                                goal_joint_state_.position);
      double max_vel_mag =
          std::max(std::fabs(current_joint_state_.velocity),
                   std::fabs(goal_joint_state_.velocity));

      // Here, we consider two cases to estimate worst case min/max
      //
      // Case 1: We move at the maximum start/end velocity for half of
      // the trajectory duration
      // Needed for cases where we do a S curve
      double potential_min_duration = min_pos - max_vel_mag * elapsed_time / 2.0;
      double potential_max_duration = max_pos + max_vel_mag * elapsed_time / 2.0;

      // Case 2: We move at the velocity needed to move from start to
      // end for half of the trajectory duration
      // Needed for cases with start and end velocity of 0
      double dist_vel_mag =
          std::fabs((goal_joint_state_.position - current_joint_state_.position)
                    / elapsed_time);
      double potential_min_duration_2 = min_pos - dist_vel_mag * elapsed_time / 2.0;
      double potential_max_duration_2 = max_pos + dist_vel_mag * elapsed_time / 2.0;

      min_pos = std::min(potential_min_duration, potential_min_duration_2);
      max_pos = std::max(potential_max_duration, potential_max_duration_2);

      EXPECT_TRUE(VerifyVectorWithinBounds(min_pos, max_pos,
                                           output_trajectory_.positions));
  }

  double calculatePositionAccuracy(KinematicState goal_joint_state,
                                   JointTrajectory& trajectory)
  {
    double goal_position = goal_joint_state_.position;
    double final_position = trajectory.positions((size() - 1));

    double error = final_position - goal_position;
    return error;
  }

  void checkPositionError()
  {
    position_error_ = calculatePositionAccuracy(
      goal_joint_state_, output_trajectory_);
    EXPECT_LT(position_error_, position_tolerance_);
  }

  void checkTimestep()
  {
    double timestep_tolerance = 0.1 * timestep_;
    EXPECT_NEAR(output_trajectory_.elapsed_times[1]
                - output_trajectory_.elapsed_times[0],
                timestep_, timestep_tolerance);
  }

  void checkDuration()
  {
    uint num_waypoint_tolerance = 1;
    uint expected_num_waypoints = 1 + desired_duration_ / timestep_;
    EXPECT_NEAR(uint(size()), expected_num_waypoints, num_waypoint_tolerance);
  }

  void verifyVelAccelJerkLimits()
  {
    double maxVelocityMagnitude = \
      output_trajectory_.velocities.cwiseAbs().maxCoeff();
    EXPECT_LE(maxVelocityMagnitude, joint_limits_.velocity_limit);
    double maxAccelerationMagnitude = \
      output_trajectory_.accelerations.cwiseAbs().maxCoeff();
    EXPECT_LE(maxAccelerationMagnitude, joint_limits_.acceleration_limit);
    double maxJerkMagnitude = \
      output_trajectory_.jerks.cwiseAbs().maxCoeff();
    EXPECT_LE(maxJerkMagnitude, joint_limits_.jerk_limit);
  }

  void writeOutputToFiles()
  {
    std::ofstream output_file;
    std::string file = BASE_FILEPATH + "_" + name() + ".csv";
    std::cerr << "Writing values to file " << file << std::endl;

    output_file.open(file, std::ofstream::out);
    for (Eigen::Index waypoint = 0; waypoint < output_trajectory_.positions.size(); ++waypoint)
    {
      output_file << output_trajectory_.elapsed_times(waypoint) << " "
                  << output_trajectory_.positions(waypoint) << " "
                  << output_trajectory_.velocities(waypoint) << " "
                  << output_trajectory_.accelerations(waypoint) << " "
                  << output_trajectory_.jerks(waypoint) << std::endl;
    }
    output_file.clear();
    output_file.close();
  }

  void runTest()
  {
    genTrajectory();
    checkPositionError();
    checkTimestep();
    checkDuration();
    verifyVelAccelJerkLimits();
    checkBounds();
  }

  void TearDown() override
  {
    if (write_output_)
      writeOutputToFiles();
  }

};  // class SingleJointGeneratorTest

TEST_F(SingleJointGeneratorTest, LimitAcceleration)
{
  // Limit only acceleration
  // Derived from LinuxCNC "limit3/limit-accel-and-max" test

  goal_joint_state_.position = 160;

  joint_limits_.velocity_limit = 1e99;
  joint_limits_.acceleration_limit = 1000;
  joint_limits_.jerk_limit = 1e99;

  desired_duration_ = 0.800;
  max_duration_ = 10.0;
  timestep_ = 0.001;

  runTest();
}

TEST_F(SingleJointGeneratorTest, LimitVelocity)
{
  // Limit only velocity
  // Derived from LinuxCNC "limit3/limit-velocity" test

  goal_joint_state_.position = 400;

  joint_limits_.velocity_limit = 500;
  joint_limits_.acceleration_limit = 1e99;
  joint_limits_.jerk_limit = 1e99;

  desired_duration_ = 0.800;
  max_duration_ = 10.0;
  timestep_ = 0.001;

  runTest();
}

}  // namespace trackjoint

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  return result;
}
