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
  NO_ERROR = 0,
  DESIRED_DURATION_TOO_SHORT = 1,
  MAX_DURATION_EXCEEDED = 2,
  VELOCITY_EXCEEDS_LIMIT = 3,
  ACCEL_EXCEEDS_LIMIT = 4,
  MAX_DURATION_LESS_THAN_DESIRED_DURATION = 5,
  LIMIT_NOT_POSITIVE = 6,
  GOAL_POSITION_MISMATCH = 7,
  FAILURE_TO_GENERATE_SINGLE_WAYPOINT = 8,
  OBJECT_NOT_RESET = 9,
  ERROR_IN_TIMESTEP_STRETCHING = 10
};

/**
 * \brief Use this map to look up human-readable strings for each error code
 */
const std::unordered_map<uint, std::string> ERROR_CODE_MAP({
    { NO_ERROR, "No error, trajectory generation was successful" },
    { DESIRED_DURATION_TOO_SHORT,
      "Desired duration is too short, cannot have less than one timestep in a "
      "trajectory" },
    { MAX_DURATION_EXCEEDED, "Max duration was exceeded" },
    { VELOCITY_EXCEEDS_LIMIT, "A velocity input exceeds the velocity limit" },
    { ACCEL_EXCEEDS_LIMIT, "An acceleration input exceeds the acceleration limit" },
    { MAX_DURATION_LESS_THAN_DESIRED_DURATION, "max_duration should not be less than desired_duration" },
    { LIMIT_NOT_POSITIVE, "Vel/accel/jerk limits should be greater than zero" },
    { GOAL_POSITION_MISMATCH, "Mismatch between the final position and the goal position" },
    { FAILURE_TO_GENERATE_SINGLE_WAYPOINT, "Failed to generate even a single new waypoint" },
    { OBJECT_NOT_RESET, "Must call reset() before generating trajectory" },
    { ERROR_IN_TIMESTEP_STRETCHING, "Error during timestep stretching" }
});
}  // end namespace trackjoint
