
/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2019, PickNik Consulting
 *  All rights reserved.
 *
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Helper functions for testing TrackJoint
*/

#pragma once

// Testing
#include <gtest/gtest.h>

// Target testing library
#include <trackjoint/trajectory_generator.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>

namespace trackjoint {
double CalculatePositionAccuracy(
    std::vector<trackjoint::KinematicState> goal_joint_states,
    std::vector<trackjoint::JointTrajectory> &trajectory) {
  // Use a 2-norm to calculate positional accuracy
  Eigen::VectorXd goal_positions(trajectory.size());
  Eigen::VectorXd final_positions(trajectory.size());

  for (size_t joint = 0; joint < trajectory.size(); ++joint) {
    goal_positions(joint) = goal_joint_states[joint].position;
    final_positions(joint) = trajectory.at(joint).positions(
        (trajectory.at(joint).positions.size() - 1));
  }

  double error = (final_positions - goal_positions).norm();

  return error;
}

void VerifyVelAccelJerkLimits(
    std::vector<trackjoint::JointTrajectory> &trajectory,
    const std::vector<trackjoint::Limits> &limits){
  for (size_t joint = 0; joint < trajectory.size(); ++joint) {
    EXPECT_LE(trajectory.at(joint).velocities.cwiseAbs().maxCoeff(), limits[joint].velocity_limit);
    EXPECT_LE(trajectory.at(joint).accelerations.cwiseAbs().maxCoeff(), limits[joint].acceleration_limit);
    EXPECT_LE(trajectory.at(joint).jerks.cwiseAbs().maxCoeff(), limits[joint].jerk_limit);
  }
}

std::vector<std::vector<double>> LoadWaypointsFromFile(const std::string &file_name) {
  std::ifstream input_file(file_name);
  std::string line;
  std::vector<std::vector<double>> v;

  std::string tempstr;
  double tempdouble;
  char delimiter;

  while (std::getline(input_file, tempstr)) {
      std::istringstream iss(tempstr);
      std::vector<double> tempv;
      while (iss >> tempdouble) {
          tempv.push_back(tempdouble);
          iss >> delimiter;
      }
      v.push_back(tempv);
  }
  return v;
}
}  // namespace trackjoint
