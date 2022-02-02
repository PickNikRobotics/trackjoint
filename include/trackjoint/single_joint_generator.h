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
   Desc: Generate jerk-limited trajectories for a single joint.
*/

#pragma once

#include <cmath>  // copysign
#include "trackjoint/error_codes.h"
#include "trackjoint/joint_trajectory.h"
#include "trackjoint/kinematic_state.h"
#include "trackjoint/configuration.h"
#include "trackjoint/limits.h"
#include "trackjoint/utilities.h"

namespace trackjoint
{
class SingleJointGenerator
{
public:
  /** \brief Constructor
   *
   * input num_waypoints_threshold minimum/maximum number of waypoints for full trajectory/streaming modes, respectively
   * input max_num_waypoints_trajectory_mode to maintain determinism, return an error if more than this many waypoints
   * is required
   */
  SingleJointGenerator(size_t num_waypoints_threshold, size_t max_num_waypoints_trajectory_mode);

  /** \brief reset data members and prepare to generate a new trajectory
   *
   * input configuration A `Configuration` object
   * input current_joint_states vector of the initial kinematic states for each degree of freedom
   * input goal_joint_states vector of the target kinematic states for each degree of freedom
   * input desired_num_waypoints nominal number of waypoints, calculated from user-supplied duration and timestep
   * input timestep_was_upsampled If upsampling happened (we are working with very few waypoints), do not adjust
   * timestep
   *
   */
  void reset(const Configuration& configuration, const KinematicState& current_joint_state,
             const KinematicState& goal_joint_state, size_t desired_num_waypoints, bool timestep_was_upsampled);

  /** \brief reset data members and prepare to generate a new trajectory
   *
   * input timestep desired time between waypoints
   * input max_duration allow the trajectory to be extended up to this limit. Error if that cannot be done.
   * input current_joint_states vector of the initial kinematic states for each degree of freedom
   * input goal_joint_states vector of the target kinematic states for each degree of freedom
   * input limits vector of kinematic limits for each degree of freedom
   * input desired_num_waypoints nominal number of waypoints, calculated from user-supplied duration and timestep
   * input position_tolerance tolerance for how close the final trajectory should follow a smooth interpolation.
   *                          Should be set lower than the accuracy requirements for your task
   * input use_streaming_mode set to true for fast streaming applications. Returns a maximum of num_waypoints_threshold
   *                          waypoints.
   * input timestep_was_upsampled If upsampling happened (we are working with very few waypoints), do not adjust
   * timestep
   *
   */
  void reset(double timestep, double max_duration, const KinematicState& current_joint_state,
             const KinematicState& goal_joint_state, const Limits& limits, size_t desired_num_waypoints,
             const double position_tolerance, bool use_streaming_mode, bool timestep_was_upsampled);

  /** \brief Generate a jerk-limited trajectory for this joint
   *
   * return a TrackJoint status code
   */
  ErrorCodeEnum generateTrajectory();

  /** \brief Extend a trajectory to a new duration. Magnitudes of vel/accel/jerk will be decreased. */
  void extendTrajectoryDuration();

  /** \brief Get the generated trajectory
   *
   * return a vector of kinematic states for the joint
   */
  JointTrajectory getTrajectory();

  /** \brief Get the last waypoint that successfully matched the polynomial interpolation
   *
   * return the waypoint index
   */
  size_t getLastSuccessfulIndex();

  /** \brief Update desired_duration_ for this joint
   *
   * input new_trajectory_duration the new desired duration. Units not important as long as they are consistent
   */
  void updateTrajectoryDuration(double new_trajectory_duration);

  /** \brief Start looking back through a velocity vector to calculate for an
   * excess velocity at limited_index. */
  bool backwardLimitCompensation(size_t limited_index, double excess_velocity);

  /** \brief This method is used to set waypoints_ state, for testing */
  void setInternalWaypointsData(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
                                const Eigen::VectorXd& accelerations, const Eigen::VectorXd& jerks,
                                const Eigen::VectorXd& elapsed_times);

private:
  /** \brief Record the index when compensation first failed */
  inline void recordFailureTime(size_t current_index, size_t* index_last_successful)
  {
    // Record the index when compensation first failed
    if (current_index < *index_last_successful)
    {
      *index_last_successful = current_index;
    }
  };

  /** \brief interpolate from start to end state with a polynomial
   *
   * input times a vector of waypoint times.
   * return a vector of interpolated positions
   */
  Eigen::VectorXd interpolate(Eigen::VectorXd& times);

  /** \brief Step through a vector of velocities, compensating for limits. Start from the beginning. */
  ErrorCodeEnum forwardLimitCompensation(size_t* index_last_successful);

  /** \brief This uses backwardLimitCompensation() but it starts from a position
   * vector */
  ErrorCodeEnum positionVectorLimitLookAhead(size_t* index_last_successful);

  /** \brief Check whether the duration needs to be extended, and do it */
  ErrorCodeEnum predictTimeToReach();

  /** \brief Calculate vel/accel/jerk from position */
  void calculateDerivativesFromPosition();

  /** \brief Calculate accel/jerk from velocity */
  void calculateDerivativesFromVelocity();

  const size_t kNumWaypointsThreshold;
  const size_t kMaxNumWaypointsFullTrajectory;

  Configuration configuration_;
  double desired_duration_;
  KinematicState current_joint_state_;
  KinematicState goal_joint_state_;
  Eigen::VectorXd nominal_times_;
  JointTrajectory waypoints_;
  size_t index_last_successful_;
  bool is_reset_;
};  // end class SingleJointGenerator
}  // namespace trackjoint
