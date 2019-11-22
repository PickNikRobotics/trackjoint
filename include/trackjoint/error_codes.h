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

const std::unordered_map<uint, std::string> kErrorCodeMap(
    {{0, "No error, trajectory generation was successful"},
     {1, "Internal error: Length mismatch between quaternion components"},
     {2, "Internal error: Length mismatch between translation and rotation"},
     {3, "Desired duration is too short, cannot have less than one timestep in a "
      "trajectory"},
     {4, "Current orientation input quaternion is not normalized"},
     {5, "Goal orientation quaternion is not normalized"},
     {6, "Max duration was exceeded"},
     {7, "A velocity input exceeds the velocity limit"},
     {8, "An acceleration input exceeds the acceleration limit"},
     {9, "max_duration should not be less than desired_duration"},
     {10, "Vel/accel/jerk limits should be greater than zero"},
     {11, "Mismatch between the final position and the goal position"}});
}  // end namespace trackjoint
