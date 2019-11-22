/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

#include <trackjoint/single_joint_generator.h>

namespace trackjoint {
SingleJointGenerator::SingleJointGenerator(
    double desired_duration, double max_duration,
    trackjoint::KinematicState &current_joint_state,
    trackjoint::KinematicState &goal_joint_state, trackjoint::Limits &limits) {
  ;
}

ErrorCodeEnum SingleJointGenerator::GenerateTrajectory() {
  Interpolate();

  PositionVectorLimitLookAhead();

  // TODO(andyz): check for duration extension

  return PredictTimeToReach();
}

void SingleJointGenerator::Interpolate() { ; }

void SingleJointGenerator::PositionVectorLimitLookAhead() { ; }

ErrorCodeEnum SingleJointGenerator::PredictTimeToReach() {
  return ErrorCodeEnum::NO_ERROR;
}

}  // end namespace trackjoint
