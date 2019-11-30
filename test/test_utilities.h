
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

namespace trackjoint
{
	double CalculatePositionAccuracy(std::vector<trackjoint::KinematicState> goal_joint_states, std::vector<trackjoint::JointTrajectory> &trajectory)
  {
    // Use a 2-norm to calculate positional accuracy
    Eigen::VectorXd goal_positions(trajectory.size());
    Eigen::VectorXd final_positions(trajectory.size());

    for (size_t joint = 0; joint < trajectory.size(); ++joint)
    {
      goal_positions(joint) = goal_joint_states[joint].position;
      final_positions(joint) = trajectory.at(joint).positions(( trajectory.at(joint).positions.size() - 1 ));
    }

    double error = (final_positions - goal_positions).norm();

    return error;
  }
}  // namespace trackjoint
