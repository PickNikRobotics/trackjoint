// Copyright 2021 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
  LESS_THAN_TEN_TIMESTEPS_FOR_STREAMING_MODE = 9,
  OBJECT_NOT_RESET = 10,
};

/**
 * \brief Use this map to look up human-readable strings for each error code
 */
// clang-format off
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
    { LESS_THAN_TEN_TIMESTEPS_FOR_STREAMING_MODE,
      "In streaming mode, desired duration should be at least 10 timesteps" },
    { OBJECT_NOT_RESET, "Must call reset() before generating trajectory" },
    // clang-format on
});
}  // end namespace trackjoint
