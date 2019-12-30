/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

#include <trackjoint/trajectory_generator.h>
#include <fstream>

namespace trackjoint
{
TrajectoryGenerator::TrajectoryGenerator(uint num_dof, double timestep, double desired_duration, double max_duration,
                                         const std::vector<KinematicState>& current_joint_states,
                                         const std::vector<KinematicState>& goal_joint_states,
                                         const std::vector<Limits>& limits)
  : kNumDof(num_dof)
  , desired_duration_(desired_duration)
  , max_duration_(max_duration)
  , kDesiredTimestep(timestep)
  ,
  // Default timestep
  upsampled_timestep_(timestep)
  , limits_(limits)
{
  // Upsample if num. waypoints would be short. Helps with accuracy
  UpSample();

  // Initialize a trajectory generator for each joint
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    single_joint_generators_.push_back(SingleJointGenerator(upsampled_timestep_, desired_duration_, max_duration_,
                                                            current_joint_states[joint], goal_joint_states[joint],
                                                            limits[joint], upsampled_num_waypoints_, kMaxNumWaypoints));
  }
}

void TrajectoryGenerator::UpSample()
{
  // Decrease the timestep to improve accuracy.
  // Upsample algorithm:
  // Keep the first and last waypoint.
  // Insert a new waypoint between every pre-existing waypoint.
  // The formula for the new number of waypoints is new_num_waypoints = 2*num_waypoints-1
  // Upsample_rounds_ tracks how many times this was applied so we can reverse it later.

  upsampled_num_waypoints_ = 1 + desired_duration_ / upsampled_timestep_;

  while (upsampled_num_waypoints_ < kMinNumWaypoints)
  {
    upsampled_num_waypoints_ = 2 * upsampled_num_waypoints_ - 1;

    upsampled_timestep_ = desired_duration_ / (upsampled_num_waypoints_ - 1);
    ++upsample_rounds_;
  }
  std::cout << "New duration: " << upsampled_timestep_ * (upsampled_num_waypoints_-1) << std::endl;
  std::cout << "New num waypoints: " << upsampled_num_waypoints_ << std::endl;
  std::cout << "Upsampled timestep: " << upsampled_timestep_ << std::endl;
}

void TrajectoryGenerator::DownSample(Eigen::VectorXd* time_vector, Eigen::VectorXd* position_vector,
  Eigen::VectorXd* velocity_vector, Eigen::VectorXd* acceleration_vector)
{
  // Need at least 2 waypoints
  if (time_vector->size() > 2)
  {
    // Eigen::VectorXd does not provide .back(), so get the final time like this:
    double final_time = (*time_vector)[time_vector->size() - 1];
    size_t new_vector_size = 1 + final_time / kDesiredTimestep;
    // Time downsampling:
    time_vector->setLinSpaced(new_vector_size, 0., final_time);
    std::cout << "Downsampled time vector size: " << time_vector->size() << std::endl;
    std::cout << "Downsampled end time: " << (*time_vector)[time_vector->size() - 1] << std::endl;

    // Determine length of position/velocity/acceleration from length of time vector:
    size_t num_elements_to_skip = position_vector->size() / time_vector->size();
    std::cout << "num_elements_to_skip: " << num_elements_to_skip << std::endl;

    Eigen::VectorXd new_positions(new_vector_size);
    Eigen::VectorXd new_velocities(new_vector_size);
    Eigen::VectorXd new_accelerations(new_vector_size);

    new_positions[0] = (*position_vector)[0];
    new_velocities[0] = (*velocity_vector)[0];
    new_accelerations[0] = (*acceleration_vector)[0];

    // Position/velocity/acceleration:
    for (size_t index = 1; index < new_vector_size; ++index)
    {
      new_positions[index] = (*position_vector)[index * num_elements_to_skip];
      new_velocities[index] = (*velocity_vector)[index * num_elements_to_skip];
      new_accelerations[index] = (*acceleration_vector)[index * num_elements_to_skip];
      std::cout << new_positions[index] << std::endl;
      std::cout << "index: " << index << std::endl;
    }
    std::cout << "Final downsampled position: " << new_positions[new_positions.size()-1] << std::endl;
    std::cout << "Final upsampled position: " << (*position_vector)[position_vector->size()-1] << std::endl;
    //std::cout << (*position_vector)[504] << std::endl;
    std::cout << (*position_vector)[position_vector->size() - 2] << std::endl;
    std::cout << (*position_vector)[position_vector->size() - 1] << std::endl;

    *position_vector = new_positions;
    *velocity_vector = new_velocities;
    *acceleration_vector = new_accelerations;
  }
}

ErrorCodeEnum TrajectoryGenerator::InputChecking(const std::vector<trackjoint::KinematicState>& current_joint_states,
                                                 const std::vector<trackjoint::KinematicState>& goal_joint_states,
                                                 const std::vector<Limits>& limits, double nominal_timestep)
{
  double rounded_duration = std::round(desired_duration_ / upsampled_timestep_) * upsampled_timestep_;

  // Need at least 1 timestep
  if (rounded_duration < nominal_timestep)
  {
    SetFinalStateToCurrentState();
    return ErrorCodeEnum::kDesiredDurationTooShort;
  }

  // Maximum duration must be equal to or longer than the nominal, goal duration
  if (max_duration_ < rounded_duration)
  {
    SetFinalStateToCurrentState();
    return ErrorCodeEnum::kMaxDurationLessThanDesiredDuration;
  }

  // Check that current vels. are less than the limits.
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    if (fabs(current_joint_states[joint].velocity) > limits[joint].velocity_limit)
    {
      SetFinalStateToCurrentState();
      return ErrorCodeEnum::kVelocityExceedsLimit;
    }

    // Check that goal vels. are less than the limits.
    if (fabs(goal_joint_states[joint].velocity) > limits[joint].velocity_limit)
    {
      SetFinalStateToCurrentState();
      return ErrorCodeEnum::kVelocityExceedsLimit;
    }

    // Check that current accels. are less than the limits.
    if (fabs(current_joint_states[joint].acceleration) > limits[joint].acceleration_limit)
    {
      SetFinalStateToCurrentState();
      return ErrorCodeEnum::kAccelExceedsLimit;
    }

    // Check that goal accels. are less than the limits.
    if (fabs(goal_joint_states[joint].acceleration) > limits[joint].acceleration_limit)
    {
      SetFinalStateToCurrentState();
      return ErrorCodeEnum::kAccelExceedsLimit;
    }

    // Check for positive limits.
    if (limits[joint].velocity_limit <= 0 || limits[joint].acceleration_limit <= 0 || limits[joint].jerk_limit <= 0)
    {
      SetFinalStateToCurrentState();
      return ErrorCodeEnum::kLimitNotPositive;
    }
  }

  return ErrorCodeEnum::kNoError;
}

void TrajectoryGenerator::SaveTrajectoriesToFile(const std::vector<JointTrajectory>& output_trajectories,
                                                 const std::string& base_filepath) const
{
  std::ofstream output_file;
  std::string output_path;

  for (size_t joint = 0; joint < output_trajectories.size(); ++joint)
  {
    output_path = base_filepath + std::to_string(joint + 1) + ".csv";
    output_file.open(output_path);
    for (size_t waypoint = 0; waypoint < output_trajectories.at(joint).positions.size(); ++waypoint)
    {
      output_file << output_trajectories.at(joint).elapsed_times(waypoint) << " "
                  << output_trajectories.at(joint).positions(waypoint) << " "
                  << output_trajectories.at(joint).velocities(waypoint) << " "
                  << output_trajectories.at(joint).accelerations(waypoint) << " "
                  << output_trajectories.at(joint).jerks(waypoint) << std::endl;
    }
    output_file.close();
    output_file.clear();
  }
}

ErrorCodeEnum TrajectoryGenerator::SynchronizeTrajComponents(std::vector<JointTrajectory>* output_trajectories)
{
  size_t longest_num_waypoints = 0;
  size_t index_of_longest_duration = 0;

  // Extend to the longest duration across all components
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    if (single_joint_generators_[joint].GetLastSuccessfulIndex() > longest_num_waypoints)
    {
      longest_num_waypoints = single_joint_generators_[joint].GetLastSuccessfulIndex();
      index_of_longest_duration = joint;
    }
  }

  // This indicates that a successful trajectory wasn't found, even when the
  // trajectory was extended to max_duration.
  if (longest_num_waypoints < (desired_duration_ / upsampled_timestep_))
  {
    SetFinalStateToCurrentState();
    return ErrorCodeEnum::kMaxDurationExceeded;
  }

  // Subtract one from longest_num_waypoints because the first index doesn't
  // count toward duration
  double new_desired_duration = (longest_num_waypoints - 1) * upsampled_timestep_;

  // If any of the component durations need to be extended, run them again
  if (new_desired_duration > desired_duration_)
  {
    for (size_t joint = 0; joint < kNumDof; ++joint)
    {
      if (joint != index_of_longest_duration)
      {
        single_joint_generators_[joint].UpdateTrajectoryDuration(new_desired_duration);
        single_joint_generators_[joint].ExtendTrajectoryDuration();
        output_trajectories->at(joint) = single_joint_generators_[joint].GetTrajectory();
      }
      // If this was the index of longest duration, don't need to re-generate a
      // trajectory
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

  return ErrorCodeEnum::kNoError;
}

void TrajectoryGenerator::SetFinalStateToCurrentState()
{
  // TODO(andyz)
  ;
}

ErrorCodeEnum TrajectoryGenerator::OutputChecking(const std::vector<JointTrajectory>& output_trajectories)
{
  for (size_t joint = 0; joint < kNumDof; ++joint)
  {
    // error if any element was greater than the limit

    // Velocity
    Eigen::Matrix<bool, Eigen::Dynamic, 1> result =
        (output_trajectories[joint].velocities.array() > limits_[joint].velocity_limit) ||
        (output_trajectories[joint].velocities.array() < -limits_[joint].velocity_limit);
    if ((result.array() != false).any())
    {
      return ErrorCodeEnum::kInternalLimitViolation;
    }

    // Acceleration
    result = (output_trajectories[joint].accelerations.array() > limits_[joint].acceleration_limit) ||
             (output_trajectories[joint].accelerations.array() < -limits_[joint].acceleration_limit);
    if ((result.array() != false).any())
    {
      return ErrorCodeEnum::kInternalLimitViolation;
    }

    // Jerk
    result = (output_trajectories[joint].jerks.array() > limits_[joint].jerk_limit) ||
             (output_trajectories[joint].jerks.array() < -limits_[joint].jerk_limit);
    if ((result.array() != false).any())
    {
      return ErrorCodeEnum::kInternalLimitViolation;
    }
  }

  return ErrorCodeEnum::kNoError;
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
    for (size_t joint = 0; joint < kNumDof; ++joint)
    {
      DownSample(&output_trajectories->at(joint).elapsed_times,
        &output_trajectories->at(joint).positions,
        &output_trajectories->at(joint).velocities,
        &output_trajectories->at(joint).accelerations);
/*
      std::cout << "Final position before DownSamplePositionVelAccel(): " << output_trajectories->at(joint).positions[ output_trajectories->at(joint).positions.size()-1 ] << std::endl;
      output_trajectories->at(joint).positions = DownSamplePositionVelAccel(output_trajectories->at(joint).positions);
      output_trajectories->at(joint).velocities = DownSamplePositionVelAccel(output_trajectories->at(joint).velocities);
      output_trajectories->at(joint).accelerations = DownSamplePositionVelAccel(output_trajectories->at(joint).accelerations);

      std::cout << "UPSAMPLE ROUNDS:" << std::endl;
      std::cout << upsample_rounds_ << std::endl;
      std::cout << "LENGTH BEFORE DOWNSAMPLING:" << std::endl;
      std::cout << output_trajectories->at(joint).elapsed_times.size() << std::endl;
      std::cout << "TIMESTEP BEFORE DOWNSAMPLING: " << std::endl;
      std::cout << output_trajectories->at(joint).elapsed_times[1] - output_trajectories->at(joint).elapsed_times[0] << std::endl;
      std::cout << "SEQUENCE BEFORE DOWNSAMPLING: " << std::endl;
      std::cout << output_trajectories->at(joint).elapsed_times[14] << "  "
      << output_trajectories->at(joint).elapsed_times[15] << "  "
      << output_trajectories->at(joint).elapsed_times[16] << "  "
      << std::endl;

      output_trajectories->at(joint).elapsed_times = DownSampleElapsedTimes(output_trajectories->at(joint).elapsed_times);

      std::cout << "===" << std::endl;
      std::cout << "LENGTH AFTER DOWNSAMPLING:" << std::endl;
      std::cout << output_trajectories->at(joint).elapsed_times.size() << std::endl;
      std::cout << "TIMESTEP AFTER DOWNSAMPLING: " << std::endl;
      std::cout << output_trajectories->at(joint).elapsed_times[1] - output_trajectories->at(joint).elapsed_times[0] << std::endl;
*/
    }

  // Check for limits before returning
  error_code = OutputChecking(*output_trajectories);

  return error_code;
}
}  // end namespace trackjoint
