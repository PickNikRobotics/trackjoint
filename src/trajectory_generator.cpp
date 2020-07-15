/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

#include "trackjoint/trajectory_generator.h"

#include <fstream>
#include <math.h>

namespace trackjoint
{
TrajectoryGenerator::TrajectoryGenerator(uint num_dof, double timestep, double desired_duration, double max_duration,
                                         const std::vector<KinematicState>& current_joint_states,
                                         const std::vector<KinematicState>& goal_joint_states,
                                         const std::vector<Limits>& limits, const double position_tolerance,
                                         bool use_streaming_mode)
  : kNumDof(num_dof)
  , desired_timestep_(timestep)
  , upsampled_timestep_(timestep)
  , desired_duration_(desired_duration)
  , max_duration_(max_duration)
  , current_joint_states_(current_joint_states)
  , limits_(limits)
  , use_streaming_mode_(use_streaming_mode)
{
  if(std::abs(remainder(desired_duration, timestep)) > REMAINDER_THRESHOLD) {
    //
    // Here we take in desired_duration and timestep, but don't enforce that
    // they are evenly divisible.
    //
    // Adjust timestep so it evenly divides into desired_duration
    //
    size_t num_waypoints = round(desired_duration / timestep);
    timestep = desired_duration / num_waypoints;
    std::cout << "!!! Modifying timestep to preserve num_waypoints: changing from " << desired_timestep_ << " to " << timestep << std::endl;
    desired_timestep_ = timestep;
    upsampled_timestep_ = timestep;
  }
  // Upsample if num. waypoints would be short. Helps with accuracy
  upsample();

  // Initialize a trajectory generator for each joint
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    single_joint_generators_.push_back(
        SingleJointGenerator(upsampled_timestep_, max_duration_, current_joint_states[joint],
                             goal_joint_states[joint], limits[joint], upsampled_num_waypoints_, kNumWaypointsThreshold,
                             kMaxNumWaypointsFullTrajectory, position_tolerance, use_streaming_mode_));
  }
}

void TrajectoryGenerator::reset(double timestep, double desired_duration, double max_duration,
                                const std::vector<KinematicState>& current_joint_states,
                                const std::vector<KinematicState>& goal_joint_states, const std::vector<Limits>& limits,
                                const double position_tolerance, bool use_streaming_mode)
{
  desired_timestep_ = timestep;
  upsampled_timestep_ = timestep;
  desired_duration_ = desired_duration;
  max_duration_ = max_duration;
  current_joint_states_ = current_joint_states;
  limits_ = limits;
  use_streaming_mode_ = use_streaming_mode;
  upsampled_num_waypoints_ = 0;
  upsample_rounds_ = 0;

  if(std::abs(remainder(desired_duration, timestep)) > REMAINDER_THRESHOLD) {
    //
    // Here we take in desired_duration and timestep, but don't enforce that
    // they are evenly divisible.
    //
    // Adjust timestep so it evenly divides into desired_duration
    //
    size_t num_waypoints = round(desired_duration / timestep);
    timestep = desired_duration / num_waypoints;
    std::cout << "!!! Modifying timestep to preserve num_waypoints: changing from " << desired_timestep_ << " to " << timestep << std::endl;
    desired_timestep_ = timestep;
    upsampled_timestep_ = timestep;
  }

  // Upsample if num. waypoints would be short. Helps with accuracy
  upsample();

  // Initialize a trajectory generator for each joint
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    single_joint_generators_[joint].reset(upsampled_timestep_, max_duration_,
                                          current_joint_states[joint], goal_joint_states[joint], limits[joint],
                                          upsampled_num_waypoints_, position_tolerance, use_streaming_mode_);
  }
}

void TrajectoryGenerator::upsample()
{
  // Decrease the timestep to improve accuracy.
  // upsample algorithm:
  // Keep the first and last waypoint.
  // Insert a new waypoint between every pre-existing waypoint.
  // The formula for the new number of waypoints is new_num_waypoints =
  // 2*num_waypoints-1
  // upsample_rounds_ tracks how many times this was applied so we can reverse
  // it later.

  //
  // TODO: This is where num_waypoints is set for streaming mode, but not
  // where I would expect it to be (as it isn't being upsampled). Refactor.
  //
  upsampled_num_waypoints_ = 1 + desired_duration_ / upsampled_timestep_;

  // streaming mode is designed to always return kNumWaypointsThreshold (or fewer, if only a few are successful)
  // So, upsample and downSample are not necessary.
  if (!use_streaming_mode_)
  {
    while (upsampled_num_waypoints_ < kNumWaypointsThreshold)
    {
      upsampled_num_waypoints_ = 2 * upsampled_num_waypoints_ - 1;

      upsampled_timestep_ = desired_duration_ / (upsampled_num_waypoints_ - 1);
      ++upsample_rounds_;
    }
  }
}

void TrajectoryGenerator::downSample(Eigen::VectorXd* time_vector, Eigen::VectorXd* position_vector,
                                     Eigen::VectorXd* velocity_vector, Eigen::VectorXd* acceleration_vector,
                                     Eigen::VectorXd* jerk_vector)
{
  // Need at least 3 waypoints
  if (time_vector->size() <= 2)
  {
    return;
  }

  // Eigen::VectorXd does not provide .back(), so get the final time like this:
  double final_time = (*time_vector)[time_vector->size() - 1];

  //
  // Is minimum vector size 2 or 3? Right now size 2 means no limits get checked
  //
  // minimum new_vector_size is three (initial waypoint and final waypoint)
  size_t new_vector_size = std::max(size_t(1 + round(final_time / desired_timestep_)), size_t(3));

  // Determine length of position/velocity/acceleration from length of time
  // vector:
  Eigen::VectorXd new_positions(new_vector_size);
  Eigen::VectorXd new_velocities(new_vector_size);
  Eigen::VectorXd new_accelerations(new_vector_size);

  // Keep the first and last elements of position/vel/accel since they should match exactly
  new_positions[0] = (*position_vector)[0];
  new_velocities[0] = (*velocity_vector)[0];
  new_accelerations[0] = (*acceleration_vector)[0];
  new_positions[new_positions.size() - 1] = (*position_vector)[position_vector->size() - 1];
  new_velocities[new_velocities.size() - 1] = (*velocity_vector)[velocity_vector->size() - 1];
  new_accelerations[new_accelerations.size() - 1] = (*acceleration_vector)[acceleration_vector->size() - 1];

  if(std::abs(remainder(desired_timestep_, upsampled_timestep_)) > REMAINDER_THRESHOLD &&
    (position_vector->size() - 1) % (new_vector_size - 1) == 0) {
    // The upsampled timestep is evenly divisible into the desired downsample rate and
    // the upsampled vector is evenly divisible into the desired vector
    // upsampled rate: 0.00625
    // upsamped num waypoints: 49
    // downsampled rate: 0.1
    // downsamped num waypoints: 4
    // 0.1 % 0.00625 = 0
    // (49-1) % (4-1) = 0
    // 0.1 * (4-1) = 0.3
    // 0.00625 * (49-1) = 0.3
    // 0.1 / 0.00625 = (49-1) / (4-1) = 16
    // This is small speed optimization

    // Find the upsample ratio
    size_t num_elements_to_skip = desired_timestep_ / upsampled_timestep_;
    // We have filled the first and last value, now fill remaining values
    for(size_t downsample_index = 1; downsample_index < new_vector_size - 1; ++downsample_index) {
      new_positions[downsample_index] = (*position_vector)[downsample_index * num_elements_to_skip];
      new_velocities[downsample_index] = (*velocity_vector)[downsample_index * num_elements_to_skip];
      new_accelerations[downsample_index] = (*acceleration_vector)[downsample_index * num_elements_to_skip];
    }

  } else {
    // Adjustments were made in single joint generator that made the num waypoints not evenly divisible
    // upsampled rate: 0.00625
    // upsamped num waypoints: 51
    // downsampled rate: 0.1
    // downsamped num waypoints: 4
    // 0.1 % 0.00625 = 0
    // 51-1 % 4-1 = 2
    // 0.1 * (4-1) = 0.3
    // 0.00625 * (51-1) = 0.3125
    // 0.1 / 0.00625 = 16
    // (51-1) / (4-1) = 16.6

    // The upsampled timestep is NOT evenly divisible into the desired downsample rate
    // Linearly interpolate the upsampled vector to get the downsampled vector
    // We have filled the first and last value, now fill remaining values
    for(size_t downsample_index = 1; downsample_index < new_vector_size - 1; ++downsample_index) {
      // The time at the downsampled index we are interpolating for
      double downsampled_time = desired_timestep_ * downsample_index;
      // The lower of the two adjacent indices in the upsampled vector which capture the downsampled time
      size_t lower_upsampled_index = floor(downsampled_time / upsampled_timestep_);
      // The time at the lower index in the upsampled vector
      double lower_upsampled_time = upsampled_timestep_ * lower_upsampled_index;
      // The fraction (0, 1) to apply to the lower index when interpolating for the downsampled value
      double lower_upsampled_fraction = 1.0 - ((downsampled_time - lower_upsampled_time) / upsampled_timestep_);
      // lower_upsampled_fraction + upper_upsampled_fraction == 1
      double upper_upsampled_fraction = 1.0 - lower_upsampled_fraction;
      // Perform interpolation
      new_positions[downsample_index] = lower_upsampled_fraction * (*position_vector)[lower_upsampled_index] + upper_upsampled_fraction * (*position_vector)[lower_upsampled_index + 1];
      new_velocities[downsample_index] = lower_upsampled_fraction * (*velocity_vector)[lower_upsampled_index] + upper_upsampled_fraction * (*velocity_vector)[lower_upsampled_index + 1];
      new_accelerations[downsample_index] = lower_upsampled_fraction * (*acceleration_vector)[lower_upsampled_index] + upper_upsampled_fraction * (*acceleration_vector)[lower_upsampled_index + 1];
    }
  }

  // Time downsampling:
  time_vector->setLinSpaced(new_vector_size, 0., final_time);
  *position_vector = new_positions;
  *velocity_vector = new_velocities;
  *acceleration_vector = new_accelerations;
  *jerk_vector = DiscreteDifferentiation(new_accelerations, desired_timestep_, 0);
}

ErrorCodeEnum TrajectoryGenerator::inputChecking(const std::vector<KinematicState>& current_joint_states,
                                                 const std::vector<KinematicState>& goal_joint_states,
                                                 const std::vector<Limits>& limits, double nominal_timestep)
{
  if (desired_duration_ > kMaxNumWaypointsFullTrajectory * upsampled_timestep_)
  {
    // Print a warning but do not exit
    std::cout << "Capping desired duration at " << kMaxNumWaypointsFullTrajectory
              << " waypoints to maintain determinism." << std::endl;
    desired_duration_ = kMaxNumWaypointsFullTrajectory * upsampled_timestep_;
  }

  if (max_duration_ > kMaxNumWaypointsFullTrajectory * upsampled_timestep_)
  {
    // Print a warning but do not exit
    std::cout << "Capping max duration at " << kMaxNumWaypointsFullTrajectory << " waypoints to maintain determinism."
              << std::endl;
    max_duration_ = kMaxNumWaypointsFullTrajectory * upsampled_timestep_;
  }

  double rounded_duration = std::round(desired_duration_ / upsampled_timestep_) * upsampled_timestep_;

  // Need at least 1 timestep
  if (rounded_duration < nominal_timestep)
  {
    return ErrorCodeEnum::kDesiredDurationTooShort;
  }

  // Maximum duration must be equal to or longer than the nominal, goal duration
  if (max_duration_ < rounded_duration)
  {
    return ErrorCodeEnum::kMaxDurationLessThanDesiredDuration;
  }

  // Check that current vels. are less than the limits.
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    if (abs(current_joint_states[joint].velocity) > limits[joint].velocity_limit)
    {
      return ErrorCodeEnum::kVelocityExceedsLimit;
    }

    // Check that goal vels. are less than the limits.
    if (abs(goal_joint_states[joint].velocity) > limits[joint].velocity_limit)
    {
      return ErrorCodeEnum::kVelocityExceedsLimit;
    }

    // Check that current accels. are less than the limits.
    if (abs(current_joint_states[joint].acceleration) > limits[joint].acceleration_limit)
    {
      return ErrorCodeEnum::kAccelExceedsLimit;
    }

    // Check that goal accels. are less than the limits.
    if (abs(goal_joint_states[joint].acceleration) > limits[joint].acceleration_limit)
    {
      return ErrorCodeEnum::kAccelExceedsLimit;
    }

    // Check for positive limits.
    if (limits[joint].velocity_limit <= 0 || limits[joint].acceleration_limit <= 0 || limits[joint].jerk_limit <= 0)
    {
      return ErrorCodeEnum::kLimitNotPositive;
    }

    // In streaming mode, the user-requested duration should be >= kNumWaypointsThreshold * timestep.
    // upsample and downSample aren't used in streaming mode.
    if (rounded_duration < kNumWaypointsThreshold * nominal_timestep && use_streaming_mode_)
    {
      return ErrorCodeEnum::kLessThanTenTimestepsForStreamingMode;
    }
  }

  return ErrorCodeEnum::kNoError;
}

void TrajectoryGenerator::saveTrajectoriesToFile(const std::vector<JointTrajectory>& output_trajectories,
                                                 const std::string& base_filepath, bool append_to_file) const
{
  std::ofstream output_file;
  std::string output_path;

  for (size_t joint = 0; joint < output_trajectories.size(); ++joint)
  {
    output_path = base_filepath + std::to_string(joint + 1) + ".csv";
    if (append_to_file)
    {
      output_file.open(output_path, std::ofstream::out | std::ofstream::app);
    }
    else
    {
      output_file.open(output_path, std::ofstream::out);
    }
    for (size_t waypoint = 0; waypoint < static_cast<size_t>(output_trajectories.at(joint).positions.size());
         ++waypoint)
    {
      output_file << output_trajectories.at(joint).elapsed_times(waypoint) << " "
                  << output_trajectories.at(joint).positions(waypoint) << " "
                  << output_trajectories.at(joint).velocities(waypoint) << " "
                  << output_trajectories.at(joint).accelerations(waypoint) << " "
                  << output_trajectories.at(joint).jerks(waypoint) << std::endl;
    }
    output_file.clear();
    output_file.close();
  }
}

ErrorCodeEnum TrajectoryGenerator::synchronizeTrajComponents(std::vector<JointTrajectory>* output_trajectories)
{
  // Normal mode: extend to the longest duration across all components
  // streaming mode: clip all components at the shortest successful number of waypoints

  // No need to synchronize if there's only one joint

  size_t longest_num_waypoints = 0;
  size_t index_of_longest_duration = 0;
  size_t shortest_num_waypoints = SIZE_MAX;

  // Find longest and shortest durations
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    // TODO: It would be better to have a function in single joint generator
    // that returned if it was successful in generating a trajectory rather
    // than doing non-intuitive calculations like this
    if (single_joint_generators_[joint].getLastSuccessfulIndex() > longest_num_waypoints)
    {
      longest_num_waypoints = single_joint_generators_[joint].getLastSuccessfulIndex() + 1;
      index_of_longest_duration = joint;
    }

    if (single_joint_generators_[joint].getLastSuccessfulIndex() < shortest_num_waypoints)
    {
      shortest_num_waypoints = single_joint_generators_[joint].getLastSuccessfulIndex() + 1;
    }
  }

  // Normal mode, extend to the longest duration so all components arrive at the same time
  if (!use_streaming_mode_)
  {
    //
    // Below is a buggy check because now that we recompute num wp to be in
    // line with timestep and desired duration, single joint generator tries
    // to do this in 1 timestep (for spline stretching, I believe), i.e. with
    // 2 waypoints.
    //
    // Spline stretching doesn't seem to have run when this check takes place
    // because longest_num_waypoints is 2 in test failures cases. If it had
    // run, I would expect enough waypoints to fill out the desired duration we
    // provided.
    //
    // Note that in the current test case failures, the trajectories are never
    // attempted to be expanded, so the error 'kMaxDurationExceeded' is incorrect
    //
    // Should this check even be here? Shouldn't kMaxDurationExceeded errors
    // be caught in predictTimeToReach and prevent synchronization from even
    // being attempted? I am commenting it out for now
    //

    // // This indicates that a successful trajectory wasn't found, even when the trajectory was extended to max_duration
    // if ((longest_num_waypoints - 1) < std::floor(desired_duration_ / upsampled_timestep_))
    // {
    //   //
    //   // This check needs to be fixed - it falsely triggers when the
    //   // SingleJointGenerator constructor bug is fixed
    //   //
    //   return ErrorCodeEnum::kMaxDurationExceeded;
    // }

    // Subtract one from longest_num_waypoints because the first index doesn't count toward duration
    double new_desired_duration = (longest_num_waypoints - 1) * upsampled_timestep_;
    bool update_and_extend_all = (kNumDof == 1);

    // If any of the component durations were changed, run them again
    if (new_desired_duration != desired_duration_)
    {
      if (new_desired_duration < desired_duration_) {
          //
          // I think the spirit of this block of code assumes new_desired_duration > desired_duration_,
          // but with the other changes in this PR this is no longer the case
          //
          // When new_desired_duration < desired_duration_, then we should be
          // doing spline interpolation to fill out the remaining time
          std::cout << "!!! new_desired_duration (" << new_desired_duration << ") < desired_duration_ (" << desired_duration_ << "): keeping desired_duration_" << std::endl;
          new_desired_duration = desired_duration_;
          update_and_extend_all = true;
      }
      for (size_t joint = 0; joint < kNumDof; ++joint)
      {
        if (update_and_extend_all || joint != index_of_longest_duration)
        {
          single_joint_generators_[joint].updateTrajectoryDuration(new_desired_duration);
          single_joint_generators_[joint].extendTrajectoryDuration();
          output_trajectories->at(joint) = single_joint_generators_[joint].getTrajectory();

          std::cout << "End position for Joint " << joint << ":  " <<
            single_joint_generators_[joint].getTrajectory().positions[single_joint_generators_[joint].getTrajectory().positions.size()-1] << std::endl;
        }
        // If this was the index of longest duration, don't need to re-generate a trajectory
        else
        {
          output_trajectories->at(joint) = single_joint_generators_[joint].getTrajectory();
        }
      }
    }
    else
    {
      for (size_t joint = 0; joint < kNumDof; ++joint)
      {
        output_trajectories->at(joint) = single_joint_generators_[joint].getTrajectory();
      }
    }
  }
  // streaming mode, clip at the shortest number of waypoints across all components
  else if (use_streaming_mode_)
  {
    for (size_t joint = 0; joint < kNumDof; ++joint)
    {
      output_trajectories->at(joint) = single_joint_generators_[joint].getTrajectory();

      ClipEigenVector(&output_trajectories->at(joint).positions, shortest_num_waypoints);
      ClipEigenVector(&output_trajectories->at(joint).velocities, shortest_num_waypoints);
      ClipEigenVector(&output_trajectories->at(joint).accelerations, shortest_num_waypoints);
      ClipEigenVector(&output_trajectories->at(joint).jerks, shortest_num_waypoints);
      ClipEigenVector(&output_trajectories->at(joint).elapsed_times, shortest_num_waypoints);
    }
  }

  return ErrorCodeEnum::kNoError;
}

void TrajectoryGenerator::clipVectorsForOutput(std::vector<JointTrajectory>* trajectory)
{
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    for (auto waypt = 0; waypt < trajectory->at(joint).velocities.size(); ++waypt)
    {
      // Velocity
      if (trajectory->at(joint).velocities[waypt] > limits_[joint].velocity_limit)
        trajectory->at(joint).velocities[waypt] = limits_[joint].velocity_limit;
      if (trajectory->at(joint).velocities[waypt] < -limits_[joint].velocity_limit)
        trajectory->at(joint).velocities[waypt] = -limits_[joint].velocity_limit;

      // Acceleration
      if (trajectory->at(joint).accelerations[waypt] > limits_[joint].acceleration_limit)
        trajectory->at(joint).accelerations[waypt] = limits_[joint].acceleration_limit;
      if (trajectory->at(joint).accelerations[waypt] < -limits_[joint].acceleration_limit)
        trajectory->at(joint).accelerations[waypt] = -limits_[joint].acceleration_limit;

      // Jerk
      if (trajectory->at(joint).jerks[waypt] > limits_[joint].jerk_limit)
        trajectory->at(joint).jerks[waypt] = limits_[joint].jerk_limit;
      if (trajectory->at(joint).jerks[waypt] < -limits_[joint].jerk_limit)
        trajectory->at(joint).jerks[waypt] = -limits_[joint].jerk_limit;
    }
  }
}

ErrorCodeEnum TrajectoryGenerator::generateTrajectories(std::vector<JointTrajectory>* output_trajectories)
{
  ErrorCodeEnum error_code = ErrorCodeEnum::kNoError;
  // Generate individual joint trajectories
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    error_code = single_joint_generators_[joint].generateTrajectory();
    if (error_code)
    {
      return error_code;
    }
  }

  // Synchronize trajectory components
  error_code = synchronizeTrajComponents(output_trajectories);
  if (error_code)
  {
    return error_code;
  }

  // downSample all vectors, if needed, to the correct timestep
  if (upsample_rounds_ > 0)
  {
    for (size_t joint = 0; joint < kNumDof; ++joint)
    {
      downSample(&output_trajectories->at(joint).elapsed_times, &output_trajectories->at(joint).positions,
                 &output_trajectories->at(joint).velocities, &output_trajectories->at(joint).accelerations,
                 &output_trajectories->at(joint).jerks);
    }
  }

  // To be on the safe side, ensure limits are obeyed
  clipVectorsForOutput(output_trajectories);

  return error_code;
}
}  // end namespace trackjoint
