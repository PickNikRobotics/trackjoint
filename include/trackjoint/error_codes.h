/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

#pragma once

#include <string>
#include <unordered_map>

namespace trackjoint {

enum ErrorCodeEnum {
  kNoError = 0,
  kQquaternionLengthMismatch = 1,
  kTranslationRotationLengthMismatch = 2,
  kDesiredDurationTooShort = 3,
  kCurrentOrientationNotNormalized = 4,
  kGoalOrientationNotNormalized = 5,
  kMaxDurationExceeded = 6,
  kVelocityExceedsLimit = 7,
  kAccelExceedsLimit = 8,
  kMaxDurationLessThanDesiredDuration = 9,
  kLimitNotPositive = 10,
  kGoalPositionMismatch = 11
};

const std::unordered_map<std::string, uint> kErrorCodeMap(
    {{"No error, trajectory generation was successful", 0},
     {"Internal error: Length mismatch between quaternion components", 1},
     {"Internal error: Length mismatch between translation and rotation", 2},
     {"Desired duration is too short, cannot have less than one timestep in a "
      "trajectory",
      3},
     {"Current orientation input quaternion is not normalized", 4},
     {"Goal orientation quaternion is not normalized", 5},
     {"Max duration was exceeded", 6},
     {"A velocity input exceeds the velocity limit", 7},
     {"An acceleration input exceeds the acceleration limit", 8},
     {"max_duration should not be less than desired_duration", 9},
     {"Vel/accel/jerk limits should be greater than zero", 10},
     {"Mismatch between the final position and the goal position", 11}});
}  // end namespace trackjoint
