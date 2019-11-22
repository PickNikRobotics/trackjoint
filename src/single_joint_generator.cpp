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
    const trackjoint::KinematicState &current_joint_state,
    const trackjoint::KinematicState &goal_joint_state, const trackjoint::Limits &limits) {
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
  return ErrorCodeEnum::kNoError;
}

}  // end namespace trackjoint
