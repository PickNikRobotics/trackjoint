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

#include <fstream>
#include "trackjoint/trajectory_generator.h"

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
  // Upsample if num. waypoints would be short. Helps with accuracy
  upsample();

  // Initialize a trajectory generator for each joint
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    single_joint_generators_.push_back(SingleJointGenerator(kNumWaypointsThreshold, kMaxNumWaypointsFullTrajectory));
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

  // Upsample if num. waypoints would be short. Helps with accuracy
  upsample();

  // Initialize a trajectory generator for each joint
  bool timestep_was_upsampled = upsample_rounds_ > 0;
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    single_joint_generators_[joint].reset(upsampled_timestep_, max_duration_, current_joint_states[joint],
                                          goal_joint_states[joint], limits[joint], upsampled_num_waypoints_,
                                          position_tolerance, use_streaming_mode_, timestep_was_upsampled);
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
  size_t num_up_sampled_elements_traversed = 0;
  // The last index taken from the beginning of the upsampled vector
  size_t last_element_pulled_from_beginning = 0;
  // The last index taken from the end of the upsampled vector
  size_t last_element_pulled_from_end = position_vector->size() - 1;

  for (size_t count = 1; num_elements_filled_in_new_vector < new_vector_size - 2; ++count)
  {
    // Update num_elements_to_skip based on:
    // (num_elements_remaining_in_upsampled_vector) / (num_elements_remaining_in_new_vector)
    num_elements_to_skip = (position_vector->size() - 2 - num_up_sampled_elements_traversed) /
                           (new_vector_size - 2 - num_elements_filled_in_new_vector);

    new_positions[count] = (*position_vector)[last_element_pulled_from_beginning + num_elements_to_skip];
    new_velocities[count] = (*velocity_vector)[last_element_pulled_from_beginning + num_elements_to_skip];
    new_accelerations[count] = (*acceleration_vector)[last_element_pulled_from_beginning + num_elements_to_skip];
    ++num_elements_filled_in_new_vector;
    num_up_sampled_elements_traversed = num_up_sampled_elements_traversed + num_elements_to_skip;
    last_element_pulled_from_beginning = last_element_pulled_from_beginning + num_elements_to_skip;

    // Count down if we need to fill more elements. Subtract two because first and last element are already filled.
    if (num_elements_filled_in_new_vector < new_vector_size - 2)
    {
      // Start filling at (end-1) because the last element is already filled
      new_positions[new_positions.size() - 1 - count] =
          (*position_vector)[last_element_pulled_from_end - num_elements_to_skip];
      new_velocities[new_velocities.size() - 1 - count] =
          (*velocity_vector)[last_element_pulled_from_end - num_elements_to_skip];
      new_accelerations[new_accelerations.size() - 1 - count] =
          (*acceleration_vector)[last_element_pulled_from_end - num_elements_to_skip];
      ++num_elements_filled_in_new_vector;
      num_up_sampled_elements_traversed = num_up_sampled_elements_traversed + num_elements_to_skip;
      last_element_pulled_from_end = last_element_pulled_from_end - num_elements_to_skip;
    }
  }

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
              << " waypoints to maintain determinism." << '\n';
    desired_duration_ = kMaxNumWaypointsFullTrajectory * upsampled_timestep_;
  }

  if (max_duration_ > kMaxNumWaypointsFullTrajectory * upsampled_timestep_)
  {
    // Print a warning but do not exit
    std::cout << "Capping max duration at " << kMaxNumWaypointsFullTrajectory << " waypoints to maintain determinism."
              << '\n';
    max_duration_ = kMaxNumWaypointsFullTrajectory * upsampled_timestep_;
  }

  double rounded_duration = std::round(desired_duration_ / upsampled_timestep_) * upsampled_timestep_;

  // Need at least 1 timestep
  if (rounded_duration < nominal_timestep)
  {
    return ErrorCodeEnum::DESIRED_DURATION_TOO_SHORT;
  }

  // Maximum duration must be equal to or longer than the nominal, goal duration
  if (max_duration_ < rounded_duration)
  {
    return ErrorCodeEnum::MAX_DURATION_LESS_THAN_DESIRED_DURATION;
  }

  // Check that current vels. are less than the limits.
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    if (abs(current_joint_states[joint].velocity) > (limits[joint].velocity_limit + kDoubleEpsilon))
    {
      return ErrorCodeEnum::VELOCITY_EXCEEDS_LIMIT;
    }

    // Check that goal vels. are less than the limits.
    if (abs(goal_joint_states[joint].velocity) > (limits[joint].velocity_limit + kDoubleEpsilon))
    {
      return ErrorCodeEnum::VELOCITY_EXCEEDS_LIMIT;
    }

    // Check that current accels. are less than the limits.
    if (abs(current_joint_states[joint].acceleration) > (limits[joint].acceleration_limit + kDoubleEpsilon))
    {
      return ErrorCodeEnum::ACCEL_EXCEEDS_LIMIT;
    }

    // Check that goal accels. are less than the limits.
    if (abs(goal_joint_states[joint].acceleration) > (limits[joint].acceleration_limit + kDoubleEpsilon))
    {
      return ErrorCodeEnum::ACCEL_EXCEEDS_LIMIT;
    }

    // Check for positive limits.
    if (limits[joint].velocity_limit <= 0 || limits[joint].acceleration_limit <= 0 || limits[joint].jerk_limit <= 0)
    {
      return ErrorCodeEnum::LIMIT_NOT_POSITIVE;
    }

    // In streaming mode, the user-requested duration should be >= kNumWaypointsThreshold * timestep.
    // upsample and downSample aren't used in streaming mode.
    if (rounded_duration < kNumWaypointsThreshold * nominal_timestep && use_streaming_mode_)
    {
      return ErrorCodeEnum::LESS_THAN_TEN_TIMESTEPS_FOR_STREAMING_MODE;
    }
  }

  return ErrorCodeEnum::NO_ERROR;
}

void TrajectoryGenerator::saveTrajectoriesToFile(const std::vector<JointTrajectory>& output_trajectories,
                                                 const std::string& base_filepath, bool append_to_file) const
{
  std::ofstream output_file;
  std::string output_path;

  // Warning if the folder does not exist
  if (!boost::filesystem::exists(base_filepath))
  {
    std::cout << "Directory " << base_filepath << " does not exist. Cannot save results." << '\n';
    return;
  }

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
    for (size_t waypoint = 0; waypoint < static_cast<size_t>(output_trajectories.at(joint).positions.size()); ++waypoint)
    {
      output_file << output_trajectories.at(joint).elapsed_times(waypoint) << " "
                  << output_trajectories.at(joint).positions(waypoint) << " "
                  << output_trajectories.at(joint).velocities(waypoint) << " "
                  << output_trajectories.at(joint).accelerations(waypoint) << " "
                  << output_trajectories.at(joint).jerks(waypoint) << '\n';
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
    // This indicates that a successful trajectory wasn't found, even when the trajectory was extended to max_duration
    if ((longest_num_waypoints - 1) < std::floor(desired_duration_ / upsampled_timestep_) && !use_streaming_mode_)
    {
      return ErrorCodeEnum::MAX_DURATION_EXCEEDED;
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
          single_joint_generators_[joint].updateTrajectoryDuration(new_desired_duration);
          single_joint_generators_[joint].extendTrajectoryDuration();
          output_trajectories->at(joint) = single_joint_generators_[joint].getTrajectory();
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

  return ErrorCodeEnum::NO_ERROR;
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
  ErrorCodeEnum error_code = ErrorCodeEnum::NO_ERROR;
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
