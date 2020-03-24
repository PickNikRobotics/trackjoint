/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: The status codes that TrackJoint can return.
*/

#pragma once

#include <string>
#include <unordered_map>

namespace trackjoint
{
/**
 * \brief TrackJoint returns integer error codes
 */
enum ErrorCodeEnum
{
  kNoError = 0,
  kDesiredDurationTooShort = 1,
  kMaxDurationExceeded = 2,
  kVelocityExceedsLimit = 3,
  kAccelExceedsLimit = 4,
  kMaxDurationLessThanDesiredDuration = 5,
  kLimitNotPositive = 6,
  kGoalPositionMismatch = 7,
  kFailureToGenerateSingleWaypoint = 8,
  kLessThanTenTimestepsForStreamingMode = 9
};

/**
 * \brief Use this map to look up human-readable strings for each error code
 */
const std::unordered_map<uint, std::string> kErrorCodeMap(
    { { kNoError, "No error, trajectory generation was successful" },
      { kDesiredDurationTooShort, "Desired duration is too short, cannot have less than one timestep in a "
                                  "trajectory" },
      { kMaxDurationExceeded, "Max duration was exceeded" },
      { kVelocityExceedsLimit, "A velocity input exceeds the velocity limit" },
      { kAccelExceedsLimit, "An acceleration input exceeds the acceleration limit" },
      { kMaxDurationLessThanDesiredDuration, "max_duration should not be less than desired_duration" },
      { kLimitNotPositive, "Vel/accel/jerk limits should be greater than zero" },
      { kGoalPositionMismatch, "Mismatch between the final position and the goal position" },
      { kFailureToGenerateSingleWaypoint, "Failed to generate even a single new waypoint" },
      { kLessThanTenTimestepsForStreamingMode, "In streaming mode, desired duration should be at least 10 "
                                               "timesteps" } });
}  // end namespace trackjoint
