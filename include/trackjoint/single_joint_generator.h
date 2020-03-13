/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: Generate jerk-limited trajectories for a single joint.
*/

#pragma once

#include <cmath>  // copysign
#include "trackjoint/error_codes.h"
#include "trackjoint/joint_trajectory.h"
#include "trackjoint/kinematic_state.h"
#include "trackjoint/limits.h"
#include "trackjoint/utilities.h"

namespace trackjoint
{
class SingleJointGenerator
{
public:
  /** \brief Constructor */
  SingleJointGenerator(double timestep, double desired_duration, double max_duration,
                       const KinematicState& current_joint_state, const KinematicState& goal_joint_state,
                       const trackjoint::Limits& limits, size_t desired_num_waypoints, size_t min_num_waypoints,
                       size_t max_num_waypoints, const double position_tolerance, bool use_high_speed_mode);

  /** \brief Generate a jerk-limited trajectory for this joint */
  ErrorCodeEnum GenerateTrajectory();

  /** \brief Calculate a trajectory once duration is known. Similar to
   * GenerateTrajectory minus PredictTimeToRead(). */
  ErrorCodeEnum ExtendTrajectoryDuration();

  /** \brief Get the generated trajectory */
  JointTrajectory GetTrajectory();

  /** \brief Get the last index that successfully matched the polynomial
   * interpolation */
  size_t GetLastSuccessfulIndex();

  /** \brief Update desired_duration_ for this joint */
  void UpdateTrajectoryDuration(double new_trajectory_duration);

private:
  /** \brief Interpolate from start to end state with a polynomial */
  Eigen::VectorXd Interpolate(Eigen::VectorXd& times);

  /** \brief Step through a vector of velocities, compensating for limits. Start from the beginning. */
  ErrorCodeEnum ForwardLimitCompensation(size_t* index_last_successful);

  /** \brief Start looking back through a velocity vector to calculate for an
   * excess velocity at limited_index. */
  bool BackwardLimitCompensation(size_t limited_index, double* excess_velocity);

  /** \brief This uses BackwardLimitCompensation() but it starts from a position
   * vector */
  ErrorCodeEnum PositionVectorLimitLookAhead(size_t* index_last_successful);

  /** \brief Record the index when compensation first failed */
  inline void RecordFailureTime(size_t current_index, size_t* index_last_successful);

  /** \brief Check whether the duration needs to be extended, and do it */
  ErrorCodeEnum PredictTimeToReach();

  /** \brief Calculate vel/accel/jerk from position */
  void CalculateDerivatives();

  const double kTimestep;
  const KinematicState kCurrentJointState;
  const trackjoint::Limits kLimits;
  const size_t kMaxNumHighSpeedWaypoints, kMaxNumWaypoints;
  const double kPositionTolerance;

  // If high-speed mode is enabled, trajectories are clipped at kMaxNumHighSpeedWaypoints so the algorithm runs quickly
  // High-speed mode is intended for realtime streaming applications.
  // There could be even fewer waypoints if the goal is very close or the algorithm only finds a few waypoints
  // successfully.
  // In high-speed mode, trajectory duration is not extended until it successfully reaches the goal.
  const bool kUseHighSpeedMode;

  KinematicState goal_joint_state_;
  double desired_duration_, max_duration_;
  Eigen::VectorXd nominal_times_;
  JointTrajectory waypoints_;
  size_t index_last_successful_;
};  // end class SingleJointGenerator
}  // namespace trackjoint
