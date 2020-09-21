/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: Generate jerk-limited trajectories for n degrees of freedom.
*/

#pragma once

// TrackJoint
#include "trackjoint/error_codes.h"
#include "trackjoint/joint_trajectory.h"
#include "trackjoint/kinematic_state.h"
#include "trackjoint/limits.h"
#include "trackjoint/single_joint_generator.h"
#include "trackjoint/utilities.h"

// C++
#include <Eigen/Geometry>
#include <memory>  // shared_ptr
#include <vector>

namespace trackjoint
{
/** \brief The top-level trajectory generation object. Stores and synchronized data for n degrees of freedom. */
class TrajectoryGenerator
{
public:
  /** \brief Constructor
   *
   * input num_dof number of degrees of freedom
   * input timestep desired time between waypoints
   * input desired_duration total desired duration of the trajectory
   * input max_duration allow the trajectory to be extended up to this limit. Error if that cannot be done.
   * input current_joint_states vector of the initial kinematic states for each degree of freedom
   * input goal_joint_states vector of the target kinematic states for each degree of freedom
   * input limits vector of kinematic limits for each degree of freedom
   * input position_tolerance tolerance for how close the final trajectory should follow a smooth interpolation.
   *                          Should be set lower than the accuracy requirements for your task
   * input use_streaming_mode set to true for fast streaming applications. Returns a maximum of kNumWaypointsThreshold
   * waypoints.
   */
  TrajectoryGenerator(uint num_dof, double timestep, double desired_duration, double max_duration,
                      const std::vector<KinematicState>& current_joint_states,
                      const std::vector<KinematicState>& goal_joint_states, const std::vector<Limits>& limits,
                      const double position_tolerance, bool use_streaming_mode);

  /** \brief reset the member variables of the object and prepare to generate a new trajectory */
  void reset(double timestep, double desired_duration, double max_duration,
             const std::vector<KinematicState>& current_joint_states,
             const std::vector<KinematicState>& goal_joint_states, const std::vector<Limits>& limits,
             const double position_tolerance, bool use_streaming_mode);

  /** \brief Generate and return trajectories for every joint
   *
   * input output_trajectories the calculated trajectories for n joints
   */
  ErrorCodeEnum generateTrajectories(std::vector<JointTrajectory>* output_trajectories);

  /** \brief Save generated trajectory to a .csv file
   *
   * input output_trajectories the calculated trajectories for n joints
   * input directory to save in
   * input append_to_file adds new data to existing file if true
   */
  void saveTrajectoriesToFile(const std::vector<JointTrajectory>& output_trajectories, const std::string& base_filepath,
                              bool append_to_file = false) const;

  /** \brief Check user input for errors
   *
   * input current_joint_states vector of the initial kinematic states for each degree of freedom
   * input goal_joint_states vector of the target kinematic states for each degree of freedom
   * input limits vector of kinematic limits for each degree of freedom
   * input nominal_timestep the user-requested time between waypoints
   * return a TrackJoint status code
   */
  ErrorCodeEnum inputChecking(const std::vector<KinematicState>& current_joint_states,
                              const std::vector<KinematicState>& goal_joint_states, const std::vector<Limits>& limits,
                              double nominal_timestep);

private:
  /** \brief Ensure limits are obeyed before outputting.
   *
   * input trajectory the calculated trajectories for n joints
   */
  void clipVectorsForOutput(std::vector<JointTrajectory>* trajectory);

  /** \brief upsample if num. waypoints would be short. Helps with accuracy. */
  void upsample();

  /** \brief Undo upsample() to output a time/position/velocity/acceleration series with the correct spacing.
   *
   * input time_vector a vector of times
   * input position_vector a vector of positions
   * input velocity_vector a vector of velocities
   * input acceleration_vector a vector of accelerations
   * input jerk_vector a vector of jerks
   */
  void downSample(Eigen::VectorXd* time_vector, Eigen::VectorXd* position_vector, Eigen::VectorXd* velocity_vector,
                  Eigen::VectorXd* acceleration_vector, Eigen::VectorXd* jerk_vector);

  /** \brief Synchronize all trajectories with the one of longest duration.
   *
   * input output_trajectories the calculated trajectories for n joints
   * returna TrackJoint status code
   */
  ErrorCodeEnum synchronizeTrajComponents(std::vector<JointTrajectory>* output_trajectories);

  // TODO(andyz): set this back to a small number when done testing
  // TODO(709): Remove kMaxNumWaypointsFullTrajectory - not needed now that we have streaming mode
  const size_t kMaxNumWaypointsFullTrajectory = 10000;  // A relatively small number, to run fast
  // Upsample for better accuracy if num waypoints is below threshold in full trajectory mode
  // Clip trajectories to threshold in streaming mode
  const size_t kNumWaypointsThreshold = 10;

  const uint kNumDof;
  double desired_timestep_, upsampled_timestep_;
  double desired_duration_, max_duration_;
  std::vector<KinematicState> current_joint_states_;
  std::vector<Limits> limits_;
  bool use_streaming_mode_;
  std::vector<SingleJointGenerator> single_joint_generators_;
  size_t upsampled_num_waypoints_;
  size_t upsample_rounds_ = 0;  // Every time we upsample, timestep is halved. Track this.
};                              // end class TrajectoryGenerator
}  // namespace trackjoint
