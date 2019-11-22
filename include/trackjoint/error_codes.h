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
// TrackJoint returns integer error codes. Use this array to look up the
// corresponding string.
const std::unordered_map<std::string, uint> error_code_map(
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

enum ErrorCodeEnum {
  NO_ERROR = 0,
  QUATERNION_LENGTH_MISMATCH = 1,
  TRANSLATION_ROTATION_LENGTH_MISMATCH = 2,
  DESIRED_DURATION_TOO_SHORT = 3,
  CURRENT_ORIENTATION_NOT_NORMALIZED = 4,
  GOAL_ORIENTATION_NOT_NORMALIZED = 5,
  MAX_DURATION_EXCEEDED = 6,
  VELOCITY_EXCEEDS_LIMIT = 7,
  ACCEL_EXCEEDS_LIMIT = 8,
  MAX_DURATION_LESS_THAN_DESIRED_DURATION = 9,
  LIMIT_NOT_POSITIVE = 10,
  GOAL_POSITION_MISMATCH = 11
};

}  // end namespace trackjoint
