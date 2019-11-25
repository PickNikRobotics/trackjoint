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

namespace trackjoint {
class SingleJointGenerator {
 public:
  /** \brief Constructor */
  SingleJointGenerator(double timestep, double desired_duration, double max_duration,
                       const KinematicState &current_joint_state,
                       const KinematicState &goal_joint_state,
                       const trackjoint::Limits &limits);

  /** \brief Generate a jerk-limited trajectory for this joint */
  ErrorCodeEnum GenerateTrajectory();

 private:
  /** \brief Interpolate from start to end state with a polynomial */
  Eigen::VectorXd Interpolate();

  /** \brief Step through a vector of velocities, compensating for limits
   *
   * \return The last successful position index.
  **/
  size_t LimitCompensation();

  /** \brief Start looking back through a velocity vector to calculate for an excess velocity at limited_index. */
  bool VelocityCompensation(size_t limited_index, double excess_velocity);

  /** \brief Record the index when compensation first failed */
  size_t RecordFailureTime(size_t current_index, size_t index_last_successful);

  /** \brief Check whether the duration needs to be extended, and do it */
  ErrorCodeEnum PredictTimeToReach();

  const double kTimestep;
  const double kDesiredDuration;
  const double kMaxDuration;
  const KinematicState kCurrentJointState;
  const KinematicState kGoalJointState;
  const trackjoint::Limits kLimits;

  Eigen::VectorXd times_;
  JointTrajectory waypoints_;
};  // end class SingleJointGenerator
}  // namespace trackjoint
