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

#include <trackjoint/error_codes.h>
#include <trackjoint/kinematic_state.h>
#include <trackjoint/limits.h>

namespace trackjoint {
class SingleJointGenerator {
 public:
  /** \brief Constructor */
  SingleJointGenerator(double desired_duration, double max_duration,
                       const KinematicState &current_joint_state,
                       const KinematicState &goal_joint_state,
                       const trackjoint::Limits &limits);

  /** \brief Generate a jerk-limited trajectory for this joint */
  ErrorCodeEnum GenerateTrajectory();

 private:
  /** \brief Interpolate from start to end state with a polynomial */
  void Interpolate();

  /** \brief Step through a vector of positions, compensating for limits*/
  void PositionVectorLimitLookAhead();

  /** \brief Check whether the duration needs to be extended, and do it */
  ErrorCodeEnum PredictTimeToReach();
};  // end class SingleJointGenerator
}  // namespace trackjoint
