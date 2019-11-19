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

/* Author: Andy Zelenak
   Desc: Generate jerk-limited trajectories for n robot joints.
*/

#pragma once

// TrackJoint
#include <trackjoint/single_joint_generator.h>

// C++
#include <memory>  // shared_ptr
#include <Eigen/Geometry>
#include <vector>

// Testing
#include <gtest/gtest.h>


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

/**
 * \brief Joint position, velocity, and acceleration
 */
struct KinematicState
{
  // State at this waypoint in a global, inertial reference frame
  double position;  // radians
  double velocity;
  double acceleration;

  /** \brief Print this state
   */
  void print();
};

/**
 * \brief A point of the output trajectory in Cartesian space, with time parameterization
 */
struct TrajectoryWaypoint
{
  // Translational and angular state of 6dof Cartesian pose for position, velocity, and acceleration
  KinematicState state;

  // Time elapsed since start of trajectory in seconds
  double elapsed_time;
};

/**
 * \brief Maximum dynamics of Cartesian pose, parameters of TrackPose
 */
struct CartesianLimits
{
  // Scalar linear velocity limit [m/s]
  double linear_velocity_limit;
  // Scalar linear acceleration limit [m/s^2]
  double linear_acceleration_limit;
  // Scalar linear jerk limit [m/s^3]
  double linear_jerk_limit;

  // Tolerance on successful velocity matching [m/s]
  double linear_velocity_tolerance;
  // Tolerance on successful acceleration matching [m/s^s]
  double linear_acceleration_tolerance;
  // Tolerance on successful jerk matching [m/s^3]
  double linear_jerk_tolerance;

  // Scalar limit on angular velocity [rad/s]
  double angular_velocity_limit;
  // Scalar limit on angular acceleration [rad/s^2]
  double angular_acceleration_limit;
  // Scalar limit on angular jerk [rad/s^3]
  double angular_jerk_limit;

  // Tolerance on successful velocity matching [rad/s]
  double angular_velocity_tolerance;
  // Tolerance on successful acceleration matching [rad/s^2]
  double angular_acceleration_tolerance;
  // Tolerance on successful jerk matching [rad/s^3]
  double angular_jerk_tolerance;
};

class TrajectoryGenerator
{
public:
  /** \brief Constructor */
  TrajectoryGenerator(const uint num_dof, const double timestep,
	const double desired_duration, const double max_duration,
	std::vector<KinematicState> &current_joint_states,
	std::vector<KinematicState> &goal_joint_states,
	std::vector<CartesianLimits> &limits, const double velocity_tolerance,
	const double acceleration_tolerance, const double jerk_tolerance);

  /** \brief Generate and return trajectories for every joint*/
  void GenerateTrajectories(std::vector<std::vector<TrajectoryWaypoint>> &output_trajectories);

protected:
  uint num_dof_;

  // --------------------------------------------------------
  // Any test that requires access to protected variables should go here
  FRIEND_TEST(TestTrajectoryGenerator, TestNameOfClass);
};  // end class TrajectoryGenerator

// Create std pointers for this class
typedef std::shared_ptr<TrajectoryGenerator> TrajectoryGeneratorPtr;
typedef std::shared_ptr<const TrajectoryGenerator> TrajectoryGeneratorConstPtr;

}  // namespace trackjoint
