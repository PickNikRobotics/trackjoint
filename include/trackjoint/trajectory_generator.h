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
  ErrorCodeEnum GenerateTrajectories(
      std::vector<JointTrajectory> *output_trajectories);

  /** \brief Save generated trajectory to a .csv file */
  void SaveTrajectoriesToFile(
      const std::vector<JointTrajectory> &output_trajectories,
      const std::string &base_filepath) const;

 private:
  /** \brief Check user input for errors */
  ErrorCodeEnum InputChecking(const std::vector<trackjoint::KinematicState> &current_joint_states,
      const std::vector<trackjoint::KinematicState> &goal_joint_states,
      const std::vector<Limits> &limits);

  /** \brief Upsample if num. waypoints would be short. Helps with accuracy. */
  void UpSample();

  /** \brief Undo UpSample() to output waypoints with the correct spacing. */
  Eigen::VectorXd DownSample(const Eigen::VectorXd &vector_to_downsample);

  /** \brief Synchronize all trajectories with the one of longest duration. */
  ErrorCodeEnum SynchronizeTrajComponents(
      std::vector<JointTrajectory> *output_trajectories);

  /** \brief Set the output state equal to the current state. Used if an error
   * is encountered. */
  void SetFinalStateToCurrentState();

  const uint kNumDof;
  double desired_duration_, max_duration_;
  // TODO(andyz): set this back to a small number when done testing
  const size_t kMaxNumWaypoints =
      10000;  // A relatively small number, to run fast
  const size_t kMinNumWaypoints =
      49;  // Upsample for better accuracy if fewer than this many waypoints
  std::vector<trackjoint::SingleJointGenerator> single_joint_generators_;
  double upsampled_timestep_, nominal_timestep_;
  size_t upsample_rounds_ =
      0;  // Every time we upsample, timestep is halved. Track this.
};        // end class TrajectoryGenerator
}  // namespace trackjoint
