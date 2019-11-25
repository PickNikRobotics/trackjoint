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
  kQuaternionLengthMismatch = 1,
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
  {{kNoError, "No error, trajectory generation was successful"},
   {kQuaternionLengthMismatch, "Internal error: Length mismatch between quaternion components"},
   {kTranslationRotationLengthMismatch, "Internal error: Length mismatch between translation and rotation"},
   {kDesiredDurationTooShort, "Desired duration is too short, cannot have less than one timestep in a "
    "trajectory"},
   {kCurrentOrientationNotNormalized, "Current orientation input quaternion is not normalized"},
   {kGoalOrientationNotNormalized, "Goal orientation quaternion is not normalized"},
   {kMaxDurationExceeded, "Max duration was exceeded"},
   {kVelocityExceedsLimit, "A velocity input exceeds the velocity limit"},
   {kAccelExceedsLimit, "An acceleration input exceeds the acceleration limit"},
   {kMaxDurationLessThanDesiredDuration, "max_duration should not be less than desired_duration"},
   {kLimitNotPositive, "Vel/accel/jerk limits should be greater than zero"},
   {kGoalPositionMismatch, "Mismatch between the final position and the goal position"}});
}  // end namespace trackjoint
