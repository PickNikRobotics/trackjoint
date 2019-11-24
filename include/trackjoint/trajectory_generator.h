/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: Generate jerk-limited trajectories for n robot joints.
*/

#pragma once

// TrackJoint
#include "trackjoint/error_codes.h"
#include "trackjoint/joint_trajectory.h"
#include "trackjoint/kinematic_state.h"
#include "trackjoint/limits.h"
#include "trackjoint/single_joint_generator.h"

// C++
#include <Eigen/Geometry>
#include <memory>  // shared_ptr
#include <vector>

// Testing
#include <gtest/gtest.h>

namespace trackjoint {

class TrajectoryGenerator {
 public:
  /** \brief Constructor */
  TrajectoryGenerator(uint num_dof, double timestep, double desired_duration,
                      double max_duration,
                      const std::vector<KinematicState> &current_joint_states,
                      const std::vector<KinematicState> &goal_joint_states,
                      const std::vector<Limits> &limits);

  /** \brief Generate and return trajectories for every joint*/
  void GenerateTrajectories(std::vector<JointTrajectory> *output_trajectories);

  /** \brief Save generated trajectory to a .csv file */
  void SaveTrajectoriesToFile(
      const std::vector<JointTrajectory> &output_trajectories,
      const std::string &base_filepath) const;

 protected:
  const uint kNumDof;
  double desired_duration_;
  uint error_code_ = ErrorCodeEnum::kNoError;
  std::vector<trackjoint::SingleJointGenerator> single_joint_generators_;
  double upsampled_timestep_;
};  // end class TrajectoryGenerator
}  // namespace trackjoint
