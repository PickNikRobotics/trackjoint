/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

#include <fstream>
#include "trackjoint/trajectory_generator.h"

namespace trackjoint
{
TrajectoryGenerator::TrajectoryGenerator(uint num_dof, double timestep, double desired_duration, double max_duration,
                                         const std::vector<KinematicState>& current_joint_states,
                                         const std::vector<KinematicState>& goal_joint_states,
                                         const std::vector<Limits>& limits, const double position_tolerance,
                                         bool use_high_speed_mode)
  : kNumDof(num_dof)
  , desired_timestep_(timestep)
  , upsampled_timestep_(timestep)
  , desired_duration_(desired_duration)
  , max_duration_(max_duration)
  , current_joint_states_(current_joint_states)
  , limits_(limits)
  , use_high_speed_mode_(use_high_speed_mode)
{
  // Upsample if num. waypoints would be short. Helps with accuracy
  UpSample();

  // Initialize a trajectory generator for each joint
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    single_joint_generators_.push_back(
        SingleJointGenerator(upsampled_timestep_, desired_duration_, max_duration_, current_joint_states[joint],
                             goal_joint_states[joint], limits[joint], upsampled_num_waypoints_, kMinNumWaypoints,
                             kMaxNumWaypoints, position_tolerance, use_high_speed_mode_));
  }
}

void TrajectoryGenerator::Reset(double timestep, double desired_duration, double max_duration,
                                const std::vector<KinematicState>& current_joint_states,
                                const std::vector<KinematicState>& goal_joint_states, const std::vector<Limits>& limits,
                                const double position_tolerance, bool use_high_speed_mode)
{
  desired_timestep_ = timestep;
  upsampled_timestep_ = timestep;
  desired_duration_ = desired_duration;
  max_duration_ = max_duration;
  current_joint_states_ = current_joint_states;
  limits_ = limits;
  use_high_speed_mode_ = use_high_speed_mode;
  upsampled_num_waypoints_ = 0;
  upsample_rounds_ = 0;

  // Upsample if num. waypoints would be short. Helps with accuracy
  UpSample();

  // Initialize a trajectory generator for each joint
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    single_joint_generators_[joint].Reset(upsampled_timestep_, desired_duration_, max_duration_,
                                          current_joint_states[joint], goal_joint_states[joint], limits[joint],
                                          upsampled_num_waypoints_, position_tolerance, use_high_speed_mode_);
  }
}

void TrajectoryGenerator::UpSample()
{
  // Decrease the timestep to improve accuracy.
  // Upsample algorithm:
  // Keep the first and last waypoint.
  // Insert a new waypoint between every pre-existing waypoint.
  // The formula for the new number of waypoints is new_num_waypoints =
  // 2*num_waypoints-1
  // Upsample_rounds_ tracks how many times this was applied so we can reverse
  // it later.

  upsampled_num_waypoints_ = 1 + desired_duration_ / upsampled_timestep_;

  // High-speed mode is designed to always return kMinNumWaypoints (or fewer, if only a few are successful)
  // So, UpSample and DownSample are not necessary.
  if (!use_high_speed_mode_)
  {
    while (upsampled_num_waypoints_ < kMinNumWaypoints)
    {
      upsampled_num_waypoints_ = 2 * upsampled_num_waypoints_ - 1;

      upsampled_timestep_ = desired_duration_ / (upsampled_num_waypoints_ - 1);
      ++upsample_rounds_;
    }
  }
}

void TrajectoryGenerator::DownSample(Eigen::VectorXd* time_vector, Eigen::VectorXd* position_vector,
                                     Eigen::VectorXd* velocity_vector, Eigen::VectorXd* acceleration_vector,
                                     Eigen::VectorXd* jerk_vector)
{
  // Need at least 2 waypoints
  if (time_vector->size() <= 2)
  {
    return;
  }

  // Eigen::VectorXd does not provide .back(), so get the final time like this:
  double final_time = (*time_vector)[time_vector->size() - 1];
  // minimum new_vector_size is two (initial waypoint and final waypoint)
  size_t new_vector_size = std::max(size_t(1 + round(final_time / desired_timestep_)), size_t(2));
  // Time downsampling:
  time_vector->setLinSpaced(new_vector_size, 0., final_time);

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

  // Position/velocity/acceleration
  // Do not replace first and last elements.
  // March inward from the first and last elements, meeting in the middle.
  // We know when to stop based on the number of elements filled.

  size_t num_elements_to_skip;

  // Total number of elements filled in the new vector
  size_t num_elements_filled_in_new_vector = 0;
  // Number of elements traversed in the upsampled vector
  size_t num_upsampled_elements_traversed = 0;
  for (size_t count = 1; num_elements_filled_in_new_vector < new_vector_size - 2; ++count)
  {
    // Update num_elements_to_skip based on:
    // (num_elements_remaining_in_upsampled_vector) /
    // (num_elements_remaining_in_new_vector)
    num_elements_to_skip = (position_vector->size() - 2 - num_upsampled_elements_traversed) /
                           (new_vector_size - 1 - num_elements_filled_in_new_vector);

    new_positions[count] = (*position_vector)[count * num_elements_to_skip];
    new_velocities[count] = (*velocity_vector)[count * num_elements_to_skip];
    new_accelerations[count] = (*acceleration_vector)[count * num_elements_to_skip];
    ++num_elements_filled_in_new_vector;
    num_upsampled_elements_traversed = num_upsampled_elements_traversed + num_elements_to_skip;

    // Count down if we need to fill more elements. Subtract two because first and last element are already filled.
    if (num_elements_filled_in_new_vector < new_vector_size - 2)
    {
      // Start filling at (end-1) because the last element is already filled
      new_positions[new_positions.size() - 1 - count] =
          (*position_vector)[position_vector->size() - 1 - count * num_elements_to_skip];
      new_velocities[new_velocities.size() - 1 - count] =
          (*velocity_vector)[velocity_vector->size() - 1 - count * num_elements_to_skip];
      new_accelerations[new_accelerations.size() - 1 - count] =
          (*acceleration_vector)[acceleration_vector->size() - 1 - count * num_elements_to_skip];
      ++num_elements_filled_in_new_vector;
      num_upsampled_elements_traversed = num_upsampled_elements_traversed + num_elements_to_skip;
    }
  }

  *position_vector = new_positions;
  *velocity_vector = new_velocities;
  *acceleration_vector = new_accelerations;
  *jerk_vector = DiscreteDifferentiation(new_accelerations, desired_timestep_, 0);
}

ErrorCodeEnum TrajectoryGenerator::InputChecking(const std::vector<trackjoint::KinematicState>& current_joint_states,
                                                 const std::vector<trackjoint::KinematicState>& goal_joint_states,
                                                 const std::vector<Limits>& limits, double nominal_timestep)
{
  if (desired_duration_ > kMaxNumWaypoints * upsampled_timestep_)
  {
    // Print a warning but do not exit
    std::cout << "Capping desired duration at " << kMaxNumWaypoints << " waypoints to maintain determinism."
              << std::endl;
    desired_duration_ = kMaxNumWaypoints * upsampled_timestep_;
  }

  if (max_duration_ > kMaxNumWaypoints * upsampled_timestep_)
  {
    // Print a warning but do not exit
    std::cout << "Capping max duration at " << kMaxNumWaypoints << " waypoints to maintain determinism." << std::endl;
    max_duration_ = kMaxNumWaypoints * upsampled_timestep_;
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

    // In high-speed mode, the user-requested duration should be >= kMinNumWaypoints * timestep.
    // UpSample and DownSample aren't used in high-speed mode.
    if (rounded_duration < kMinNumWaypoints * nominal_timestep && use_high_speed_mode_)
    {
      return ErrorCodeEnum::kLessThanTenTimestepsForHighSpeedMode;
    }
  }

  return ErrorCodeEnum::kNoError;
}

void TrajectoryGenerator::SaveTrajectoriesToFile(const std::vector<JointTrajectory>& output_trajectories,
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

ErrorCodeEnum TrajectoryGenerator::SynchronizeTrajComponents(std::vector<JointTrajectory>* output_trajectories)
{
  // Normal mode: extend to the longest duration across all components
  // High-speed mode: clip all components at the shortest successful number of waypoints

  // No need to synchronize if there's only one joint

  size_t longest_num_waypoints = 0;
  size_t index_of_longest_duration = 0;
  size_t shortest_num_waypoints = SIZE_MAX;
  size_t index_of_shortest_duration = 0;

  // Find longest and shortest durations
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    if (single_joint_generators_[joint].GetLastSuccessfulIndex() > longest_num_waypoints)
    {
      longest_num_waypoints = single_joint_generators_[joint].GetLastSuccessfulIndex() + 1;
      index_of_longest_duration = joint;
    }

    if (single_joint_generators_[joint].GetLastSuccessfulIndex() < shortest_num_waypoints)
    {
      shortest_num_waypoints = single_joint_generators_[joint].GetLastSuccessfulIndex() + 1;
      index_of_shortest_duration = joint;
    }
  }

  // Normal mode, extend to the longest duration so all components arrive at the same time
  if (!use_high_speed_mode_)
  {
    // This indicates that a successful trajectory wasn't found, even when the trajectory was extended to max_duration
    if ((longest_num_waypoints - 1) < (desired_duration_ / upsampled_timestep_) && !use_high_speed_mode_)
    {
      return ErrorCodeEnum::kMaxDurationExceeded;
    }

    // Subtract one from longest_num_waypoints because the first index doesn't count toward duration
    double new_desired_duration = (longest_num_waypoints - 1) * upsampled_timestep_;

    // If any of the component durations were changed, run them again
    if (new_desired_duration != desired_duration_)
    {
      for (size_t joint = 0; joint < kNumDof; ++joint)
      {
        if (joint != index_of_longest_duration)
        {
          single_joint_generators_[joint].UpdateTrajectoryDuration(new_desired_duration);
          single_joint_generators_[joint].ExtendTrajectoryDuration();
          output_trajectories->at(joint) = single_joint_generators_[joint].GetTrajectory();
        }
        // If this was the index of longest duration, don't need to re-generate a trajectory
        else
        {
          output_trajectories->at(joint) = single_joint_generators_[joint].GetTrajectory();
        }
      }
    }
    else
    {
      for (size_t joint = 0; joint < kNumDof; ++joint)
      {
        output_trajectories->at(joint) = single_joint_generators_[joint].GetTrajectory();
      }
    }
  }
  // High-speed mode, clip at the shortest number of waypoints across all components
  else if (use_high_speed_mode_)
  {
    for (size_t joint = 0; joint < kNumDof; ++joint)
    {
      output_trajectories->at(joint) = single_joint_generators_[joint].GetTrajectory();

      ClipEigenVector(&output_trajectories->at(joint).positions, shortest_num_waypoints);
      ClipEigenVector(&output_trajectories->at(joint).velocities, shortest_num_waypoints);
      ClipEigenVector(&output_trajectories->at(joint).accelerations, shortest_num_waypoints);
      ClipEigenVector(&output_trajectories->at(joint).jerks, shortest_num_waypoints);
      ClipEigenVector(&output_trajectories->at(joint).elapsed_times, shortest_num_waypoints);
    }
  }

  return ErrorCodeEnum::kNoError;
}

void TrajectoryGenerator::ClipVectorsForOutput(std::vector<JointTrajectory>* trajectory)
{
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    for (size_t joint = 0; joint < trajectory->size(); ++joint)
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
}

ErrorCodeEnum TrajectoryGenerator::GenerateTrajectories(std::vector<JointTrajectory>* output_trajectories)
{
  ErrorCodeEnum error_code = ErrorCodeEnum::kNoError;
  // Generate individual joint trajectories
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    error_code = single_joint_generators_[joint].GenerateTrajectory();
    if (error_code)
    {
      return error_code;
    }
  }

  // Synchronize trajectory components
  error_code = SynchronizeTrajComponents(output_trajectories);
  if (error_code)
  {
    return error_code;
  }

  // Downsample all vectors, if needed, to the correct timestep
  if (upsample_rounds_ > 0)
  {
    for (size_t joint = 0; joint < kNumDof; ++joint)
    {
      DownSample(&output_trajectories->at(joint).elapsed_times, &output_trajectories->at(joint).positions,
                 &output_trajectories->at(joint).velocities, &output_trajectories->at(joint).accelerations,
                 &output_trajectories->at(joint).jerks);
    }
  }

  // To be on the safe side, ensure limits are obeyed
  ClipVectorsForOutput(output_trajectories);

  return error_code;
}
}  // end namespace trackjoint
