/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

#include <trackjoint/trajectory_generator.h>
#include <fstream>

namespace trackjoint {
TrajectoryGenerator::TrajectoryGenerator(
    uint num_dof, double timestep, double desired_duration, double max_duration,
    const std::vector<KinematicState> &current_joint_states,
    const std::vector<KinematicState> &goal_joint_states,
    const std::vector<Limits> &limits)
    : kNumDof(num_dof),
      desired_duration_(desired_duration),
      max_duration_(max_duration),
      // Default timestep
      upsampled_timestep_(timestep) {
  // Input error checking
  InputChecking();

  // Upsample if num. waypoints would be short. Helps with accuracy
  UpSample();

  // Initialize a trajectory generator for each joint
  for (size_t joint = 0; joint < kNumDof; ++joint) {
    single_joint_generators_.push_back(SingleJointGenerator(
        upsampled_timestep_, desired_duration_, max_duration_,
        current_joint_states[joint], goal_joint_states[joint], limits[joint],
        kMaxNumWaypoints));
  }
}

void TrajectoryGenerator::UpSample() {
  // Halve the timestep until there are at least kMinNumWaypoints

  size_t num_waypoints = 1 + desired_duration_ / upsampled_timestep_;

  while (num_waypoints < kMinNumWaypoints) {
    upsampled_timestep_ = 0.5 * upsampled_timestep_;
    ++upsample_rounds_;
    num_waypoints = 1 + desired_duration_ / upsampled_timestep_;
  }
}

Eigen::VectorXd TrajectoryGenerator::DownSample(
    const Eigen::VectorXd &vector_to_downsample) {
  Eigen::VectorXd downsampled_vector = vector_to_downsample;
  size_t expected_size = 0;
  size_t downsampled_index = 0;

  // Remove every other sample from the vector for each round of sampling
  // e.g. 1,2,3,4,5,6 gets downsampled to 1,3,5
  for (size_t round = 0; round < upsample_rounds_; ++round) {
    Eigen::VectorXd temp_vector = downsampled_vector;
    expected_size = 1 + floor((downsampled_vector.size() - 1) / 2);
    downsampled_vector.resize(expected_size);
    downsampled_vector[0] = temp_vector[0];
    if (temp_vector.size() > 3) downsampled_index = 3;
    for (size_t upsampled_index = 3; upsampled_index < temp_vector.size();
         upsampled_index = upsampled_index + 2) {
      downsampled_vector[downsampled_index] = temp_vector[upsampled_index];
      ++downsampled_index;
    }
  }

  return downsampled_vector;
}

ErrorCodeEnum TrajectoryGenerator::InputChecking() {
  ErrorCodeEnum error_code = ErrorCodeEnum::kNoError;

  if (desired_duration_ > kMaxNumWaypoints * upsampled_timestep_) {
    // Print a warning but do not exit
    std::cout << "Capping desired duration at " << kMaxNumWaypoints <<" waypoints to maintain determinism." << std::endl;
    desired_duration_ = kMaxNumWaypoints * upsampled_timestep_;
  }

  if (max_duration_ > kMaxNumWaypoints * upsampled_timestep_) {
    // Print a warning but do not exit
    std::cout << "Capping max duration at " << kMaxNumWaypoints <<" waypoints to maintain determinism." << std::endl;
    max_duration_ = kMaxNumWaypoints * upsampled_timestep_;
  }
}

void TrajectoryGenerator::SaveTrajectoriesToFile(
    const std::vector<JointTrajectory> &output_trajectories,
    const std::string &base_filepath) const {
  std::ofstream output_file;
  std::string output_path;

  for (size_t joint = 0; joint < output_trajectories.size(); ++joint) {
    output_path = base_filepath + std::to_string(joint + 1) + ".csv";
    output_file.open(output_path);
    for (size_t waypoint = 0;
         waypoint < output_trajectories.at(joint).positions.size();
         ++waypoint) {
      output_file << output_trajectories.at(joint).elapsed_times(waypoint)
                  << " " << output_trajectories.at(joint).positions(waypoint)
                  << " " << output_trajectories.at(joint).velocities(waypoint)
                  << " "
                  << output_trajectories.at(joint).accelerations(waypoint)
                  << " "
                  << output_trajectories.at(joint).jerks(waypoint)
                  << std::endl;
    }
    output_file.close();
    output_file.clear();
  }
}

ErrorCodeEnum TrajectoryGenerator::SynchronizeTrajComponents(std::vector<JointTrajectory> *output_trajectories)
{
  size_t longest_num_waypoints = 0;
  size_t index_of_longest_duration = 0;

  // Extend to the longest duration across all components
  for (size_t joint = 0; joint < kNumDof; ++joint) {
    if (single_joint_generators_[joint].GetLastSuccessfulIndex() >
        longest_num_waypoints)
    {
      longest_num_waypoints =
          single_joint_generators_[joint].GetLastSuccessfulIndex();
      index_of_longest_duration = joint;
    }
  }

  // This indicates that a successful trajectory wasn't found, even when the
  // trajectory was extended to max_duration.
  if (longest_num_waypoints < (desired_duration_ / upsampled_timestep_)) {
    SetFinalStateToCurrentState();
    return ErrorCodeEnum::kMaxDurationExceeded;
  }

  // Subtract one from longest_num_waypoints because the first index doesn't
  // count toward duration
  double new_desired_duration =
      (longest_num_waypoints - 1) * upsampled_timestep_;

  // If any of the component durations need to be extended, run them again
  if (new_desired_duration > desired_duration_) {
    for (size_t joint = 0; joint < kNumDof; ++joint) {
      if (joint != index_of_longest_duration)
      {
        single_joint_generators_[joint].UpdateTrajectoryDuration(
            new_desired_duration);
        single_joint_generators_[joint].ExtendTrajectoryDuration();
        output_trajectories->at(joint) =
            single_joint_generators_[joint].GetTrajectory();
      }
      // If this was the index of longest duration, don't need to re-generate a trajectory
      else
      {
        output_trajectories->at(joint) =
          single_joint_generators_[joint].GetTrajectory();
      }
    }
  } else {
    for (size_t joint = 0; joint < kNumDof; ++joint) {
      output_trajectories->at(joint) =
          single_joint_generators_[joint].GetTrajectory();
    }
  }

  return ErrorCodeEnum::kNoError;
}

void TrajectoryGenerator::SetFinalStateToCurrentState()
{
  // TODO(andyz)
  ;
}

ErrorCodeEnum TrajectoryGenerator::GenerateTrajectories(std::vector<JointTrajectory> *output_trajectories) {
  ErrorCodeEnum error_code = ErrorCodeEnum::kNoError;
  // Generate individual joint trajectories
  for (size_t joint = 0; joint < kNumDof; ++joint) {
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
    for (size_t joint = 0; joint < kNumDof; ++joint) {
      output_trajectories->at(joint).positions =
          DownSample(output_trajectories->at(joint).positions);
      output_trajectories->at(joint).velocities =
          DownSample(output_trajectories->at(joint).velocities);
      output_trajectories->at(joint).accelerations =
          DownSample(output_trajectories->at(joint).accelerations);
      output_trajectories->at(joint).elapsed_times =
          DownSample(output_trajectories->at(joint).elapsed_times);
    }

  // TODO(andyz): Final error checking

  return error_code;
}
}  // end namespace trackjoint
