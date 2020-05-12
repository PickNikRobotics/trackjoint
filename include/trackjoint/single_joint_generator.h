/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: Generate jerk-limited trajectories for a single joint.
*/

#pragma once

#include <cmath>  // copysign
#include "trackjoint/error_codes.h"
#include "trackjoint/joint_trajectory.h"
#include "trackjoint/kinematic_state.h"
#include "trackjoint/limits.h"
#include "trackjoint/utilities.h"

namespace trackjoint
{
class SingleJointGenerator
{
public:
  /** \brief Constructor
   *
   * input timestep desired time between waypoints
   * input desired_duration total desired duration of the trajectory
   * input max_duration allow the trajectory to be extended up to this limit. Error if that cannot be done.
   * input current_joint_states vector of the initial kinematic states for each degree of freedom
   * input goal_joint_states vector of the target kinematic states for each degree of freedom
   * input limits vector of kinematic limits for each degree of freedom
   * input desired_num_waypoints nominal number of waypoints, calculated from user-supplied duration and timestep
   * input num_waypoints_threshold minimum/maximum number of waypoints for full trajectory/streaming modes, respectively
   * input max_num_waypoints_trajectory_mode to maintain determinism, return an error if more than this many waypoints
   * is required
   * input position_tolerance tolerance for how close the final trajectory should follow a smooth interpolation.
   *                          Should be set lower than the accuracy requirements for your task
   * input use_streaming_mode set to true for fast streaming applications. Returns a maximum of num_waypoints_threshold
   * waypoints.
   */
  SingleJointGenerator(double timestep, double desired_duration, double max_duration,
                       const KinematicState& current_joint_state, const KinematicState& goal_joint_state,
                       const Limits& limits, size_t desired_num_waypoints, size_t num_waypoints_threshold,
                       size_t max_num_waypoints_trajectory_mode, const double position_tolerance,
                       bool use_streaming_mode);

  /** \brief reset data members and prepare to generate a new trajectory */
  void reset(double timestep, double desired_duration, double max_duration, const KinematicState& current_joint_state,
             const KinematicState& goal_joint_state, const Limits& limits, size_t desired_num_waypoints,
             const double position_tolerance, bool use_streaming_mode);

  /** \brief Generate a jerk-limited trajectory for this joint
   *
   * return a TrackJoint status code
   */
  ErrorCodeEnum generateTrajectory();

  /** \brief Calculate a trajectory once duration is known. Similar to generateTrajectory minus predictTimeToReach().
   *
   * return a TrackJoint status code
   */
  ErrorCodeEnum extendTrajectoryDuration();

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

  /** Run the BackwardLimitCompensation unit test
   * return the difference between intended distance traveled and distance traveled after compensation
   */
  double runBackwardLimitCompensationTest()
  {
    size_t num_steps = 21;
    // The intended velocity:
    Eigen::VectorXd intended_velocity;
    intended_velocity.resize(num_steps);
    // Element 17 is over the velocity limit
    intended_velocity << 0.997, 0.998, 0.999, 0.99987, 0.99988, 0.99989, 0.99990, 0.99991, 0.99992, 0.99993, 0.99994,
                         0.99995, 0.99996, 0.99997, 0.99998, 0.99999, 1.0, 1.00001, 1.00001, 1.00001, 1.00001;

    waypoints_.velocities.resize(num_steps);
    // Element 17 has been set to the limit (1.0), backwardLimitCompensation should make up for this
    waypoints_.velocities << 0.997, 0.998, 0.999, 0.99987, 0.99988, 0.99989, 0.99990, 0.99991, 0.99992, 0.99993,
                             0.99994, 0.99995, 0.99996, 0.99997, 0.99998, 0.99999, 1.0, 1.0 /* changed */, 1.00001, 1.00001,
                             1.00001;
    waypoints_.positions.resize(num_steps);
    waypoints_.accelerations.resize(num_steps);
    waypoints_.jerks.resize(num_steps);
    size_t limited_index = 17;
    double delta_v = waypoints_.velocities[limited_index] - intended_velocity[limited_index];

    backwardLimitCompensation(limited_index, -delta_v);

    // Check that sum(intended_velocities) == sum(compensated_velocities)
    // This is equivalent to testing that the correct distance was traveled
    if (!(intended_velocity.sum() - waypoints_.velocities.sum()))
      return false;

    // Now do a negative version of the same test
    // Element 17 is under the velocity limit
    intended_velocity << -0.997, -0.998, -0.999, -0.99987, -0.99988, -0.99989, -0.99990, -0.99991, -0.99992, -0.99993, -0.99994,
                         -0.99995, -0.99996, -0.99997, -0.99998, -0.99999, -1.0, -1.00001, -1.00001, -1.00001, -1.00001;
    // Element 17 has been set to the limit (-1.0), backwardLimitCompensation should make up for this
    waypoints_.velocities << -0.997, -0.998, -0.999, -0.99987, -0.99988, -0.99989, -0.99990, -0.99991, -0.99992, -0.99993,
                             -0.99994, -0.99995, -0.99996, -0.99997, -0.99998, -0.99999, -1.0, -1.0 /* changed */, -1.00001,
                             -1.00001, -1.00001;
    waypoints_.positions.resize(num_steps);
    waypoints_.accelerations.resize(num_steps);
    waypoints_.jerks.resize(num_steps);
    delta_v = waypoints_.velocities[limited_index] - intended_velocity[limited_index];

    backwardLimitCompensation(limited_index, -delta_v);
    std::cout << waypoints_.velocities << std::endl;

    return (intended_velocity.sum() - waypoints_.velocities.sum());
  }

private:
  /** \brief interpolate from start to end state with a polynomial
   *
   * input times a vector of waypoint times.
   * return a vector of interpolated positions
   */
  Eigen::VectorXd interpolate(Eigen::VectorXd& times);

  /** \brief Step through a vector of velocities, compensating for limits. Start from the beginning. */
  ErrorCodeEnum forwardLimitCompensation(size_t* index_last_successful);

  /** \brief Start looking back through a velocity vector to calculate for an
   * excess velocity at limited_index. */
  bool backwardLimitCompensation(size_t limited_index, double excess_velocity);

  /** \brief This uses backwardLimitCompensation() but it starts from a position
   * vector */
  ErrorCodeEnum positionVectorLimitLookAhead(size_t* index_last_successful);

  /** \brief Record the index when compensation first failed */
  inline void recordFailureTime(size_t current_index, size_t* index_last_successful);

  /** \brief Check whether the duration needs to be extended, and do it */
  ErrorCodeEnum predictTimeToReach();

  /** \brief Calculate vel/accel/jerk from position */
  void calculateDerivativesFromPosition();

  /** \brief Calculate accel/jerk from velocity */
  void calculateDerivativesFromVelocity();

  const size_t kNumWaypointsThreshold, kMaxNumWaypointsFullTrajectory;

  double timestep_;
  double desired_duration_, max_duration_;
  KinematicState current_joint_state_;
  KinematicState goal_joint_state_;
  Limits limits_;
  double position_tolerance_;
  Eigen::VectorXd nominal_times_;
  JointTrajectory waypoints_;
  size_t index_last_successful_;

  // If streaming mode is enabled, trajectories are clipped at kNumWaypointsThreshold so the algorithm runs very
  // quickly.
  // streaming mode is intended for realtime streaming applications.
  // There could be even fewer waypoints than that if the goal is very close or the algorithm only finds a few waypoints
  // successfully.
  // In streaming mode, trajectory duration is not extended until it successfully reaches the goal.
  bool use_streaming_mode_;
};  // end class SingleJointGenerator
}  // namespace trackjoint
