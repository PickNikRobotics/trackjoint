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

#include <cmath>                      // copysign
#include <unsupported/Eigen/Splines>  // Spline-fitting is used to extend trajectory duration
#include "trackjoint/error_codes.h"
#include "trackjoint/joint_trajectory.h"
#include "trackjoint/kinematic_state.h"
#include "trackjoint/configuration.h"
#include "trackjoint/limits.h"
#include "trackjoint/utilities.h"

namespace trackjoint
{
typedef Eigen::Spline<double, 1, 2> Spline1D;
typedef Eigen::SplineFitting<Spline1D> SplineFitting1D;

class SingleJointGenerator
{
public:
  /** \brief Constructor
   *
   * input num_waypoints_threshold minimum/maximum number of waypoints
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
   * input timestep_was_upsampled If upsampling happened (we are working with very few waypoints), do not adjust
   * timestep
   *
   */
  void reset(double timestep, double max_duration, const KinematicState& current_joint_state,
             const KinematicState& goal_joint_state, const Limits& limits, size_t desired_num_waypoints,
             const double position_tolerance, bool timestep_was_upsampled);

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

  /** \brief Get the number of waypoints */
  size_t getTrajectoryLength();

  /** \brief Update desired_duration_ for this joint
   *
   * input new_trajectory_duration the new desired duration. Units not important as long as they are consistent
   */
  void updateTrajectoryDuration(double new_trajectory_duration);

private:
  /** \brief interpolate from start to end state with a polynomial
   *
   * input times a vector of waypoint times.
   * return a vector of interpolated positions
   */
  Eigen::VectorXd interpolate(Eigen::VectorXd& times);

  /** \brief Step through a vector of velocities, compensating for limits. Start from the beginning. */
  ErrorCodeEnum forwardLimitCompensation(bool& successful_limit_comp);

  /** \brief Start looking back through a velocity vector to calculate for an
   * excess velocity at limited_index. */
  bool backwardLimitCompensation(size_t limited_index, double excess_velocity);

  /** \brief This uses backwardLimitCompensation() but it starts from a position
   * vector */
  ErrorCodeEnum positionVectorLimitLookAhead(bool& successful_limit_comp);

  /** \brief Check whether the duration needs to be extended, and do it */
  ErrorCodeEnum predictTimeToReach();

  /** \brief Calculate vel/accel/jerk from position */
  void calculateDerivativesFromPosition();

  const size_t kNumWaypointsThreshold, kMaxNumWaypointsFullTrajectory;

  Configuration configuration_;
  double desired_duration_;
  KinematicState current_joint_state_;
  KinematicState goal_joint_state_;
  Eigen::VectorXd nominal_times_;
  JointTrajectory waypoints_;
  bool successful_limit_comp_;
  bool is_reset_;
};  // end class SingleJointGenerator
}  // namespace trackjoint
