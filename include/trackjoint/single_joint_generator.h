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

namespace trackjoint
{
class SingleJointGenerator
{
public:
  /** \brief Constructor */
  SingleJointGenerator(double timestep, double desired_duration, double max_duration,
                       const KinematicState& current_joint_state, const KinematicState& goal_joint_state,
                       const trackjoint::Limits& limits, int desired_num_waypoints, int max_num_waypoints);

  /** \brief Generate a jerk-limited trajectory for this joint */
  ErrorCodeEnum GenerateTrajectory();

  /** \brief Calculate a trajectory once duration is known. Similar to
   * GenerateTrajectory minus PredictTimeToRead(). */
  ErrorCodeEnum ExtendTrajectoryDuration();

  /** \brief Get the generated trajectory */
  JointTrajectory GetTrajectory();

  /** \brief Get the last index that successfully matched the polynomial
   * interpolation */
  int GetLastSuccessfulIndex();

  /** \brief Update desired_duration_ for this joint */
  void UpdateTrajectoryDuration(double new_trajectory_duration);

private:
  /** \brief Interpolate from start to end state with a polynomial */
  Eigen::VectorXd Interpolate(Eigen::VectorXd& times);

  /** \brief Step through a vector of velocities, compensating for limits */
  ErrorCodeEnum LimitCompensation(int* index_last_successful);

  /** \brief Start looking back through a velocity vector to calculate for an
   * excess velocity at limited_index. */
  bool VelocityCompensation(int limited_index, double excess_velocity);

  /** \brief This uses VelocityCompensation() but it starts from a position
   * vector */
  ErrorCodeEnum PositionVectorLimitLookAhead(int* index_last_successful);

  /** \brief Record the index when compensation first failed */
  void RecordFailureTime(int current_index, int* index_last_successful);

  /** \brief Check whether the duration needs to be extended, and do it */
  ErrorCodeEnum PredictTimeToReach();

  /** \brief Calculate vel/accel/jerk from position */
  void CalculateDerivatives();

  const double kTimestep;
  const KinematicState kCurrentJointState;
  const KinematicState kGoalJointState;
  const trackjoint::Limits kLimits;
  const int kMaxNumWaypoints;

  double desired_duration_, max_duration_;
  Eigen::VectorXd nominal_times_;
  JointTrajectory waypoints_;
  int index_last_successful_;
};  // end class SingleJointGenerator
}  // namespace trackjoint
