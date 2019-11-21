/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <string>

namespace trackjoint
{
// TrackJoint returns integer error codes. Use this array to look up the corresponding string.
const std::string ERROR_CODES[17] = {
  "No error, trajectory generation was successful", "Internal error: Length mismatch between quaternion components",
  "Internal error: Length mismatch between translation and rotation",
  "Desired duration is too short, cannot have less than one timestep in a trajectory",
  "Current orientation input quaternion is not normalized", "Goal orientation quaternion is not normalized",
  // CURRENT_AND_GOAL_STATES_IDENTICAL was removed
  "Max duration was exceeded", "A translational velocity input exceeds the translational velocity limit",
  "An angular velocity input exceeds the angular velocity limit",
  "A translational acceleration input exceeds the translational velocity limit",
  "An angular acceleration input exceeds the angular velocity limit",
  "max_duration should not be less than desired_duration",
  "Translational vel/accel/jerk limits should be greater than zero",
  "Angular vel/accel/jerk limits should be greater than zero",
  "Mismatch between the final position and the goal position",
  "Mismatch between the final orientation and the goal orientation"
};

enum ErrorCodeEnum
{
  NO_ERROR = 0,
  QUATERNION_LENGTH_MISMATCH = 1,
  TRANSLATION_ROTATION_LENGTH_MISMATCH = 2,
  DESIRED_DURATION_TOO_SHORT = 3,
  CURRENT_ORIENTATION_NOT_NORMALIZED = 4,
  GOAL_ORIENTATION_NOT_NORMALIZED = 5,
  // CURRENT_AND_GOAL_STATES_IDENTICAL was removed
  MAX_DURATION_EXCEEDED = 7,
  TRANSLATIONAL_VELOCITY_EXCEEDS_LIMIT = 8,
  ANGULAR_VELOCITY_EXCEEDS_LIMIT = 9,
  TRANSLATIONAL_ACCEL_EXCEEDS_LIMIT = 10,
  ANGULAR_ACCEL_EXCEEDS_LIMIT = 11,
  MAX_DURATION_LESS_THAN_DESIRED_DURATION = 12,
  TRANSLATIONAL_LIMIT_NOT_POSITIVE = 13,
  ANGULAR_LIMIT_NOT_POSITIVE = 14,
  GOAL_POSITION_MISMATCH = 15,
  GOAL_ORIENTATION_MISMATCH = 16
};

}  // end namespace trackjoint
