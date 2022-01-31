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

/* Author: Dave Coleman
   Desc:   Adversarial testing of TrackJoint
*/

// C++
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <string>

// Testing
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

// Target testing library
#include <trackjoint/trajectory_generator.h>
#include <trackjoint/utilities.h>

// Preparing data file handling
#include <fstream>

namespace
{
std::string REF_PATH = ament_index_cpp::get_package_share_directory("trackjoint");
std::string BASE_FILEPATH = REF_PATH + "/test/data/tj_output";
constexpr double DOUBLE_TOLERANCE = 1e-9;  // For double comparison
}  // namespace

namespace trackjoint
{
class TrajectoryGenerationTest : public ::testing::Test
{
public:
  TrajectoryGenerationTest() : output_trajectories_(num_dof_), skip_teardown_checks_(false)
  {
    // Default test parameters for 3 joints
    KinematicState joint_state;
    joint_state.position = -1;
    joint_state.velocity = 0;
    joint_state.acceleration = 0;
    current_joint_states_.push_back(joint_state);
    current_joint_states_.push_back(joint_state);
    current_joint_states_.push_back(joint_state);

    joint_state.position = -0.995;
    joint_state.velocity = 0;
    joint_state.acceleration = 0;
    goal_joint_states_.push_back(joint_state);
    goal_joint_states_.push_back(joint_state);
    goal_joint_states_.push_back(joint_state);

    Limits single_joint_limits;
    single_joint_limits.velocity_limit = 20;
    single_joint_limits.acceleration_limit = 200;
    single_joint_limits.jerk_limit = 20000;
    limits_.push_back(single_joint_limits);
    limits_.push_back(single_joint_limits);
    limits_.push_back(single_joint_limits);
  }

protected:
  // Default test parameters for 3 joints
  double timestep_ = 0.01;
  double desired_duration_ = 1;
  size_t num_dof_ = 3;
  double max_duration_ = 10;
  std::vector<KinematicState> current_joint_states_, goal_joint_states_;
  std::vector<Limits> limits_;
  double position_tolerance_ = 1e-4;
  bool use_streaming_mode_ = false;
  bool write_output_ = true;
  std::vector<JointTrajectory> output_trajectories_;
  bool skip_teardown_checks_;

  void checkBounds()
  {
    for (size_t i = 0; i < output_trajectories_.size(); ++i)
    {
      if (output_trajectories_[i].elapsed_times.size() == 0)
      {
        continue;
      }
      double elapsed_time = output_trajectories_[i].elapsed_times[output_trajectories_[i].elapsed_times.size() - 1];

      // Get estimate of min/max position and velocity using start and end states
      double min_pos = std::min(current_joint_states_[i].position, goal_joint_states_[i].position);
      double max_pos = std::max(current_joint_states_[i].position, goal_joint_states_[i].position);
      double max_vel_mag =
          std::max(std::fabs(current_joint_states_[i].velocity), std::fabs(goal_joint_states_[i].velocity));

      // Here, we consider two cases to estimate worst case min/max
      // Case 1: We move at the maximum start/end velocity for half of the trajectory duration
      // Needed for cases where we do a S curve
      double potential_min = min_pos - max_vel_mag * elapsed_time / 2.0;
      double potential_max = max_pos + max_vel_mag * elapsed_time / 2.0;

      // Case 2: We move at the velocity needed to move from start to end for half of the trajectory duration
      // Needed for cases with start and end velocity of 0
      double dist_vel_mag =
          std::fabs((goal_joint_states_[i].position - current_joint_states_[i].position) / elapsed_time);
      double potential_min_2 = min_pos - dist_vel_mag * elapsed_time / 2.0;
      double potential_max_2 = max_pos + dist_vel_mag * elapsed_time / 2.0;

      min_pos = std::min(potential_min, potential_min_2);
      max_pos = std::max(potential_max, potential_max_2);

      EXPECT_TRUE(VerifyVectorWithinBounds(min_pos, max_pos, output_trajectories_[i].positions));
    }
  }

  void writeOutputToFiles()
  {
    std::ofstream output_file;
    std::string output_path;
    for (size_t joint = 0; joint < output_trajectories_.size(); ++joint)
    {
      std::string file = BASE_FILEPATH + "_" + ::testing::UnitTest::GetInstance()->current_test_info()->name() +
                         "_joint" + std::to_string(joint + 1) + ".csv";

      output_file.open(file, std::ofstream::out);
      for (size_t waypoint = 0; waypoint < static_cast<size_t>(output_trajectories_.at(0).positions.size()); ++waypoint)
      {
        output_file << output_trajectories_.at(joint).elapsed_times(waypoint) << " "
                    << output_trajectories_.at(joint).positions(waypoint) << " "
                    << output_trajectories_.at(joint).velocities(waypoint) << " "
                    << output_trajectories_.at(joint).accelerations(waypoint) << " "
                    << output_trajectories_.at(joint).jerks(waypoint) << std::endl;
      }
      output_file.clear();
      output_file.close();
    }
  }

  double calculatePositionAccuracy(std::vector<KinematicState> goal_joint_states,
                                   std::vector<JointTrajectory>& trajectory)
  {
    // Use a 2-norm to calculate positional accuracy
    Eigen::VectorXd goal_positions(trajectory.size());
    Eigen::VectorXd final_positions(trajectory.size());

    for (size_t joint = 0; joint < trajectory.size(); ++joint)
    {
      goal_positions(joint) = goal_joint_states[joint].position;
      final_positions(joint) = trajectory.at(joint).positions((trajectory.at(joint).positions.size() - 1));
    }

    // Make clang-tidy happy about taking the norm() of a zero-length
    // vector
    Eigen::VectorXd errors = final_positions - goal_positions;
    if (errors.size() < 1)
      return 0;

    double error = (errors).norm();

    return error;
  }

  void verifyVelAccelJerkLimits(std::vector<JointTrajectory>& trajectory, const std::vector<Limits>& limits)
  {
    for (size_t joint = 0; joint < trajectory.size(); ++joint)
    {
      EXPECT_LE(trajectory.at(joint).velocities.cwiseAbs().maxCoeff(), limits[joint].velocity_limit);
      EXPECT_LE(trajectory.at(joint).accelerations.cwiseAbs().maxCoeff(), limits[joint].acceleration_limit);
      EXPECT_LE(trajectory.at(joint).jerks.cwiseAbs().maxCoeff(), limits[joint].jerk_limit);
    }
  }

  std::vector<std::vector<double>> loadWaypointsFromFile(const std::string& file_name)
  {
    std::ifstream input_file(file_name);
    std::string line;
    std::vector<std::vector<double>> waypoint_vector;

    std::string tempstr;
    double tempdouble;
    char delimiter;

    while (std::getline(input_file, tempstr))
    {
      std::istringstream input_stream(tempstr);
      std::vector<double> tempv;
      while (input_stream >> tempdouble)
      {
        tempv.push_back(tempdouble);
        input_stream >> delimiter;
      }
      waypoint_vector.push_back(tempv);
    }
    return waypoint_vector;
  }

  void TearDown() override
  {
    if (!skip_teardown_checks_)
    {
      checkBounds();
      if (write_output_)
      {
        writeOutputToFiles();
      }
    }
  }

};  // class TrajectoryGenerationTest

TEST_F(TrajectoryGenerationTest, BackwardLimitCompensation)
{
  // Test SingleJointGenerator::backwardLimitCompensation()
  // The default test input is no motion
  size_t num_waypoints = 6;
  Eigen::VectorXd zero_positions(num_waypoints);
  zero_positions << 0, 0, 0, 0, 0, 0;
  Eigen::VectorXd elapsed_times(num_waypoints);
  elapsed_times << 0, 0.01, 0.02, 0.03, 0.04, 0.05;

  //////////////////////////////////////
  // Test whether jerk limits are obeyed
  //////////////////////////////////////
  // Request a higher velocity at the second-to-last timestep (index 4).
  // Jerk in the previous timestep should increase since jerk[4] is a maximum already.
  // The full correction for the velocity error is made at index 3.

  // Need to adjust some kinematic limits for the contrived test conditions to work
  limits_.at(0).acceleration_limit = 1e5;
  limits_.at(0).jerk_limit = 1e6;

  // Initialize the trajectory generator object
  SingleJointGenerator single_joint_gen(2 /* min num waypoints */, 10000 /* max num waypoints */);
  // Set the configuration of single_joint_gen
  bool timestep_was_upsampled = false;
  single_joint_gen.reset(timestep_, max_duration_, current_joint_states_.at(0), goal_joint_states_.at(0), limits_.at(0),
                         num_waypoints, position_tolerance_, use_streaming_mode_, timestep_was_upsampled);

  double velocity_error = 0.001;  // just a reasonable value, achievable in one timestep.
  // jerk[4] is already at the limit, so jerk[3] should be adjusted to correct for the velocity error
  Eigen::VectorXd input_jerk(num_waypoints);
  double jerk_4 = limits_.at(0).jerk_limit;
  input_jerk << 0, 0, 0, 0, jerk_4, 0;
  Eigen::VectorXd input_acceleration(num_waypoints);
  double accel_4 = jerk_4 * timestep_;
  input_acceleration << 0, 0, 0, 0, accel_4, accel_4 + input_jerk(5) * timestep_;
  Eigen::VectorXd input_velocity(num_waypoints);
  double vel_4 = input_acceleration(4) * timestep_ + 0.5 * input_jerk(4) * pow(timestep_, 2);
  input_velocity << 0, 0, 0, 0, vel_4,
      vel_4 + input_acceleration(5) * timestep_ + 0.5 * input_jerk(5) * pow(timestep_, 2);
  single_joint_gen.setInternalWaypointsData(zero_positions, input_velocity, input_acceleration, input_jerk,
                                            elapsed_times);
  // For this test, say the velocity at this index needs correction
  size_t limited_index = num_waypoints - 2;  // second-from-last
  single_joint_gen.backwardLimitCompensation(limited_index, velocity_error);
  // jerk[3] should be different than it was previously
  // and velocity[3] should match the target
  JointTrajectory output = single_joint_gen.getTrajectory();
  // These input quantities shouldn't have changed
  EXPECT_NEAR(output.velocities[0], input_velocity[0], DOUBLE_TOLERANCE);
  EXPECT_NEAR(output.velocities[1], input_velocity[1], DOUBLE_TOLERANCE);
  EXPECT_NEAR(output.velocities[2], input_velocity[2], DOUBLE_TOLERANCE);
  EXPECT_NEAR(output.accelerations[0], input_acceleration[0], DOUBLE_TOLERANCE);
  EXPECT_NEAR(output.accelerations[1], input_acceleration[1], DOUBLE_TOLERANCE);
  EXPECT_NEAR(output.accelerations[2], input_acceleration[2], DOUBLE_TOLERANCE);
  EXPECT_NEAR(output.jerks[0], input_jerk[0], DOUBLE_TOLERANCE);
  EXPECT_NEAR(output.jerks[1], input_jerk[1], DOUBLE_TOLERANCE);
  EXPECT_NEAR(output.jerks[2], input_jerk[2], DOUBLE_TOLERANCE);
  // velocity[3] should now match the desired value
  EXPECT_NEAR(output.velocities[3], input_velocity[3] + velocity_error, DOUBLE_TOLERANCE);
  // jerk[3] should be different than it was previously
  EXPECT_NE(output.jerks[3], input_jerk[3]);
  // index 4 and onward should not have changed
  // But, vel[4] gets clipped to the velocity limit at the very end of backwardLimitComp()
  // because the input value was too high.
  // That's fine for the purpose of this test
  // EXPECT_NEAR(output.velocities[4], input_velocity[4], DOUBLE_TOLERANCE);
  EXPECT_NEAR(output.velocities[5], input_velocity[5], DOUBLE_TOLERANCE);
  EXPECT_NEAR(output.accelerations[4], input_acceleration[4], DOUBLE_TOLERANCE);
  EXPECT_NEAR(output.accelerations[5], input_acceleration[5], DOUBLE_TOLERANCE);
  EXPECT_NEAR(output.jerks[4], input_jerk[4], DOUBLE_TOLERANCE);
  EXPECT_NEAR(output.jerks[5], input_jerk[5], DOUBLE_TOLERANCE);

  // TearDown() should skip post-test checks
  skip_teardown_checks_ = true;
}

TEST_F(TrajectoryGenerationTest, DetectNoReset)
{
  // Calling `generateTrajectories()` without an initial `reset()`
  // should return an error code (and not segfault!)

  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration_, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  ErrorCodeEnum error_code = traj_gen.generateTrajectories(&output_trajectories_);
  EXPECT_EQ(error_code, ErrorCodeEnum::OBJECT_NOT_RESET);

  // Ensure OBJECT_NOT_RESET can be converted to string
  std::cout << "Error: " << trackjoint::ERROR_CODE_MAP.at(error_code) << std::endl;

  // TearDown() should skip post-test checks
  skip_teardown_checks_ = true;
}

TEST_F(TrajectoryGenerationTest, EasyDefaultTrajectory)
{
  // Use the class defaults. This trajectory is easy, does not require limit
  // compensation or trajectory extension

  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration_, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep_, desired_duration_, max_duration_, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  // Position error
  const double position_tolerance = 1e-4;
  const double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Duration
  uint num_waypoint_tolerance = 1;
  uint expected_num_waypoints = 1 + desired_duration_ / timestep_;
  EXPECT_NEAR(uint(output_trajectories_[0].positions.size()), expected_num_waypoints, num_waypoint_tolerance);
}

TEST_F(TrajectoryGenerationTest, OneTimestepDuration)
{
  // Request a duration of one timestep

  const double desired_duration = 1 * timestep_;
  const double max_duration = 1 * timestep_;

  KinematicState joint_state;
  joint_state.position = 0;
  joint_state.velocity = 0.1;
  joint_state.acceleration = 0;
  current_joint_states_[0] = joint_state;
  current_joint_states_[1] = joint_state;
  current_joint_states_[2] = joint_state;

  joint_state.position = 0.001;
  joint_state.velocity = 0.1;
  joint_state.acceleration = 0;
  goal_joint_states_[0] = joint_state;
  goal_joint_states_[1] = joint_state;
  goal_joint_states_[2] = joint_state;

  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 20;
  single_joint_limits.jerk_limit = 2000;
  limits_.clear();
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);

  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration, max_duration, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep_, desired_duration, max_duration, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  traj_gen.generateTrajectories(&output_trajectories_);

  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  // Position error
  double position_tolerance = 1e-4;
  double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  double timestep_tolerance = 0.1 * timestep_;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep_,
              timestep_tolerance);
  // Duration
  const double duration_tolerance = 5e-3;
  size_t vector_length = output_trajectories_[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times(vector_length), desired_duration, duration_tolerance);
}

TEST_F(TrajectoryGenerationTest, RoughlyTwoTimestepDuration)
{
  // Request a duration of approximately two timesteps

  const double timestep = 0.01;
  const double desired_duration = 0.019;
  const double max_duration = desired_duration;

  KinematicState joint_state;
  joint_state.position = -0.998;
  goal_joint_states_[0] = joint_state;
  goal_joint_states_[1] = joint_state;
  goal_joint_states_[2] = joint_state;

  TrajectoryGenerator traj_gen(num_dof_, timestep, desired_duration, max_duration, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep, desired_duration, max_duration, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  traj_gen.generateTrajectories(&output_trajectories_);

  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  // Position error
  const double position_tolerance = 1e-4;
  const double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  double timestep_tolerance = 0.003;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep,
              timestep_tolerance);
  // Duration
  const double duration_tolerance = 5e-4;
  size_t vector_length = output_trajectories_[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times(vector_length), desired_duration, duration_tolerance);
  // Number of waypoints
  EXPECT_EQ(output_trajectories_[0].elapsed_times.size(), 3);
}

TEST_F(TrajectoryGenerationTest, FourTimestepDuration)
{
  // Request a duration of just four timesteps

  const double timestep = 0.01;
  const double desired_duration = 4 * timestep;
  const double max_duration = desired_duration;

  KinematicState joint_state;
  joint_state.position = -0.998;
  goal_joint_states_[0] = joint_state;
  goal_joint_states_[1] = joint_state;
  goal_joint_states_[2] = joint_state;

  TrajectoryGenerator traj_gen(num_dof_, timestep, desired_duration, max_duration, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep, desired_duration, max_duration, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  traj_gen.generateTrajectories(&output_trajectories_);

  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  // Position error
  const double position_tolerance = 1e-4;
  const double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  double timestep_tolerance = 0.1 * timestep;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep,
              timestep_tolerance);
  // Duration
  const double duration_tolerance = 5e-3;
  size_t vector_length = output_trajectories_[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times(vector_length), desired_duration, duration_tolerance);
}

TEST_F(TrajectoryGenerationTest, SixTimestepDuration)
{
  // Request a duration of six timesteps

  const double desired_duration = 6 * timestep_;
  const double max_duration = desired_duration;

  KinematicState joint_state;
  joint_state.position = 0;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  current_joint_states_[0] = joint_state;
  current_joint_states_[1] = joint_state;
  current_joint_states_[2] = joint_state;

  joint_state.position = 0.0001;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  goal_joint_states_[0] = joint_state;
  goal_joint_states_[1] = joint_state;
  goal_joint_states_[2] = joint_state;

  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 20;
  single_joint_limits.jerk_limit = 2000;
  limits_.clear();
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);

  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration, max_duration, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep_, desired_duration, max_duration, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  traj_gen.generateTrajectories(&output_trajectories_);

  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  // Position error
  double position_tolerance = 1e-4;
  double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  double timestep_tolerance = 0.1 * timestep_;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep_,
              timestep_tolerance);
  // Duration
  const double duration_tolerance = 5e-3;
  size_t vector_length = output_trajectories_[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times(vector_length), desired_duration, duration_tolerance);
}

TEST_F(TrajectoryGenerationTest, VelAccelJerkLimit)
{
  // Velocity, acceleration and jerk limits are hit

  const double max_duration = 20;

  KinematicState joint_state;
  current_joint_states_[0] = joint_state;
  current_joint_states_[1] = joint_state;
  current_joint_states_[2] = joint_state;

  goal_joint_states_[0] = joint_state;
  goal_joint_states_[1] = joint_state;
  goal_joint_states_[2] = joint_state;

  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 0.001;
  single_joint_limits.acceleration_limit = 0.0005;
  single_joint_limits.jerk_limit = 0.001;
  limits_.clear();
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);

  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep_, desired_duration_, max_duration, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  traj_gen.generateTrajectories(&output_trajectories_);

  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  verifyVelAccelJerkLimits(output_trajectories_, limits_);

  // Position error
  double position_tolerance = 1e-4;
  double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  double timestep_tolerance = 0.1 * timestep_;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep_,
              timestep_tolerance);
  // Duration
  size_t vector_length = output_trajectories_[0].elapsed_times.size() - 1;
  EXPECT_LE(output_trajectories_[0].elapsed_times(vector_length), desired_duration_);
}

TEST_F(TrajectoryGenerationTest, NoisyStreamingCommand)
{
  // Incoming command is a noisy sine wave

  timestep_ = 0.1;
  desired_duration_ = timestep_;
  max_duration_ = 10;
  const size_t num_waypoints = 500;

  std::default_random_engine random_generator;
  std::normal_distribution<double> random_distribution(2.0, 1.5);

  KinematicState joint_state;
  joint_state.position = 0;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  current_joint_states_[0] = joint_state;
  current_joint_states_[1] = joint_state;
  current_joint_states_[2] = joint_state;
  goal_joint_states_[0] = joint_state;
  goal_joint_states_[1] = joint_state;
  goal_joint_states_[2] = joint_state;

  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 15;
  single_joint_limits.jerk_limit = 200;
  limits_[0] = single_joint_limits;
  limits_[1] = single_joint_limits;
  limits_[2] = single_joint_limits;

  // For recording actual followed trajectory
  std::vector<JointTrajectory> recorded_trajectories(num_dof_);
  for (size_t joint = 0; joint < num_dof_; ++joint)
  {
    // Resize vector
    recorded_trajectories[joint].positions.resize(num_waypoints);
    recorded_trajectories[joint].velocities.resize(num_waypoints);
    recorded_trajectories[joint].accelerations.resize(num_waypoints);
    recorded_trajectories[joint].jerks.resize(num_waypoints);
    recorded_trajectories[joint].elapsed_times.resize(num_waypoints);
    // Set initial waypoint
    recorded_trajectories[joint].positions(0) = current_joint_states_[joint].position;
    recorded_trajectories[joint].velocities(0) = current_joint_states_[joint].velocity;
    recorded_trajectories[joint].accelerations(0) = current_joint_states_[joint].acceleration;
    recorded_trajectories[joint].jerks(0) = 0;
    recorded_trajectories[joint].elapsed_times(0) = 0;
  }

  Eigen::VectorXd x_desired(num_waypoints);
  Eigen::VectorXd x_smoothed(num_waypoints);

  double time = 0;
  // Create Trajectory Generator object
  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration_, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);

  for (size_t waypoint = 0; waypoint < num_waypoints; ++waypoint)
  {
    time = waypoint * timestep_;

    joint_state.position = 0.1 * sin(time) + 0.05 * random_distribution(random_generator);

    goal_joint_states_[0] = joint_state;
    goal_joint_states_[1] = joint_state;
    goal_joint_states_[2] = joint_state;

    x_desired(waypoint) = goal_joint_states_[0].position;

    traj_gen.reset(timestep_, desired_duration_, max_duration_, current_joint_states_, goal_joint_states_, limits_,
                   position_tolerance_, use_streaming_mode_);
    traj_gen.generateTrajectories(&output_trajectories_);

    verifyVelAccelJerkLimits(output_trajectories_, limits_);

    // Save the first waypoint in x_smoothed...
    x_smoothed(waypoint) = output_trajectories_.at(0).positions(1);
    // ... and setting the next current position as the updated x_smoothed
    joint_state.position = x_smoothed(waypoint);
    joint_state.velocity = output_trajectories_.at(0).velocities(1);
    joint_state.acceleration = output_trajectories_.at(0).accelerations(1);

    // Record next point
    for (size_t joint = 0; joint < num_dof_; joint++)
    {
      recorded_trajectories[joint].positions(waypoint) = output_trajectories_[joint].positions(1);
      recorded_trajectories[joint].velocities(waypoint) = output_trajectories_[joint].velocities(1);
      recorded_trajectories[joint].accelerations(waypoint) = output_trajectories_[joint].accelerations(1);
      recorded_trajectories[joint].jerks(waypoint) = output_trajectories_[joint].jerks(1);
      recorded_trajectories[joint].elapsed_times(waypoint) = time;
    }

    current_joint_states_[0] = joint_state;
    current_joint_states_[1] = joint_state;
    current_joint_states_[2] = joint_state;
  }
  EXPECT_EQ(x_desired.size(), x_smoothed.size());
  // Duration
  uint num_waypoint_tolerance = 1;
  uint expected_num_waypoints = num_waypoints;
  EXPECT_NEAR(uint(x_smoothed.size()), expected_num_waypoints, num_waypoint_tolerance);

  // Put recorded trajectories where the tearDown() method will check them
  output_trajectories_ = recorded_trajectories;
}

// TEST_F(TrajectoryGenerationTest, OscillatingUR5TrackJointCase)
//{
//  // This test comes from a MoveIt trajectory.
//  // It was successful but there was a position oscillation.
//
//  num_dof_ = 6;
//  max_duration_ = 10;
//  timestep_ = 0.0075;
//
//  current_joint_states_.resize(num_dof_);
//  limits_.resize(num_dof_);
//  output_trajectories_.resize(num_dof_);
//
//  Limits single_joint_limits;
//  single_joint_limits.velocity_limit = 0.5;
//  single_joint_limits.acceleration_limit = 2;
//  single_joint_limits.jerk_limit = 100;
//  limits_[0] = single_joint_limits;
//  limits_[1] = single_joint_limits;
//  limits_[2] = single_joint_limits;
//  limits_[3] = single_joint_limits;
//  limits_[4] = single_joint_limits;
//  limits_[5] = single_joint_limits;
//
//  ////////////////////////////////////////////////
//  // Get TrackJoint initial states and goal states
//  ////////////////////////////////////////////////
//
//  // Vector of start states - for each waypoint, for each joint
//  std::vector<std::vector<KinematicState>> trackjt_current_joint_states;
//  // Vector of goal states - for each waypoint, for each joint
//  std::vector<std::vector<KinematicState>> trackjt_goal_joint_states;
//  // Vector of goal states - for each waypoint
//  std::vector<double> trackjt_desired_durations;
//
//  KinematicState joint_state;
//
//  std::vector<std::vector<double>> moveit_des_positions;
//  std::vector<std::vector<double>> moveit_des_velocities;
//  std::vector<std::vector<double>> moveit_des_accelerations;
//  std::vector<std::vector<double>> moveit_times_from_start;
//
//  // Reading MoveIt experimental data from .txt files
//  moveit_des_positions = loadWaypointsFromFile(REF_PATH +
//  "/test/data/30_percent_speed_oscillation/moveit_des_pos.txt"); moveit_des_velocities =
//  loadWaypointsFromFile(REF_PATH +
//                                                "/test/data/30_percent_speed_oscillation/"
//                                                "moveit_des_vel.txt");
//  moveit_des_accelerations =
//      loadWaypointsFromFile(REF_PATH + "/test/data/30_percent_speed_oscillation/moveit_des_acc.txt");
//  moveit_times_from_start =
//      loadWaypointsFromFile(REF_PATH + "/test/data/30_percent_speed_oscillation/moveit_time_from_start.txt");
//
//  // For each MoveIt waypoint
//  for (std::size_t point = 0; point < moveit_times_from_start.size() - 1; ++point)
//  {
//    current_joint_states_.clear();
//    goal_joint_states_.clear();
//
//    // for each joint
//    for (size_t joint = 0; joint < num_dof_; ++joint)
//    {
//      // Save the start state of the robot
//      joint_state.position = moveit_des_positions[point][joint];
//      joint_state.velocity = moveit_des_velocities[point][joint];
//      joint_state.acceleration = moveit_des_accelerations[point][joint];
//
//      current_joint_states_.push_back(joint_state);
//
//      // Save the goal state of the robot
//      joint_state.position = moveit_des_positions[point + 1][joint];
//      joint_state.velocity = moveit_des_velocities[point + 1][joint];
//      joint_state.acceleration = moveit_des_accelerations[point + 1][joint];
//
//      goal_joint_states_.push_back(joint_state);
//    }
//
//    trackjt_current_joint_states.push_back(current_joint_states_);
//    trackjt_goal_joint_states.push_back(goal_joint_states_);
//    trackjt_desired_durations.push_back(moveit_times_from_start[point + 1][0] - moveit_times_from_start[point][0]);
//  }
//
//  // Create trajectory generator object
//  TrajectoryGenerator traj_gen(num_dof_, timestep_, trackjt_desired_durations[0], max_duration_,
//                               trackjt_current_joint_states[0], trackjt_goal_joint_states[0], limits_,
//                               position_tolerance_, use_streaming_mode_);
//
//  // Step through the saved waypoints and smooth them with TrackJoint
//  for (std::size_t point = 0; point < trackjt_desired_durations.size(); ++point)
//  {
//    traj_gen.reset(timestep_, trackjt_desired_durations[point], max_duration_, trackjt_current_joint_states[point],
//                   trackjt_goal_joint_states[point], limits_, position_tolerance_, use_streaming_mode_);
//    output_trajectories_.resize(num_dof_);
//
//    ErrorCodeEnum error_code = traj_gen.generateTrajectories(&output_trajectories_);
//
//    // Saving Trackjoint output to .csv files for plotting
//    traj_gen.saveTrajectoriesToFile(output_trajectories_, BASE_FILEPATH + "_joint");
//
//    EXPECT_EQ(ErrorCodeEnum::NO_ERROR, error_code);
//    // Timestep
//    const double timestep_tolerance = 0.25 * timestep_;
//    EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep_,
//                timestep_tolerance);
//  }
//  EXPECT_EQ(trackjt_current_joint_states.at(0).size(), trackjt_goal_joint_states.at(0).size());
//}

TEST_F(TrajectoryGenerationTest, SuddenChangeOfDirection)
{
  // Test a "corner" trajectory.
  // Velocity flips from (0.01,0,0) to (0,0,-0.01)

  const double desired_duration = 4 * timestep_;
  const double max_duration = 1;

  KinematicState joint_state;
  joint_state.position = 0;
  joint_state.velocity = 0.01;
  joint_state.acceleration = 0;
  current_joint_states_[0] = joint_state;
  joint_state.velocity = 0;
  current_joint_states_[1] = joint_state;
  current_joint_states_[2] = joint_state;

  joint_state.position = 0.01;
  joint_state.velocity = 0;
  goal_joint_states_[0] = joint_state;
  joint_state.position = 0;
  goal_joint_states_[1] = joint_state;
  joint_state.position = -0.01;
  joint_state.velocity = -0.01;
  goal_joint_states_[2] = joint_state;

  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 20;
  single_joint_limits.acceleration_limit = 200;
  single_joint_limits.jerk_limit = 20000;
  limits_.clear();
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);

  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration, max_duration, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep_, desired_duration, max_duration, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  traj_gen.generateTrajectories(&output_trajectories_);

  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  // Position error
  double position_tolerance = 1e-4;
  double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  double timestep_tolerance = 0.1 * timestep_;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep_,
              timestep_tolerance);
  // Duration
  const double duration_tolerance = 5e-3;
  size_t vector_length = output_trajectories_[0].elapsed_times.size() - 1;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times(vector_length), desired_duration, duration_tolerance);
}

TEST_F(TrajectoryGenerationTest, LimitCompensation)
{
  // These test parameters require limit compensation

  KinematicState joint_state;
  joint_state.position = -1;
  joint_state.velocity = -0.1;
  joint_state.acceleration = 0;
  current_joint_states_[0] = joint_state;
  current_joint_states_[1] = joint_state;
  current_joint_states_[2] = joint_state;

  joint_state.position = 3;
  joint_state.velocity = 1.9;
  joint_state.acceleration = 0;
  goal_joint_states_[0] = joint_state;
  goal_joint_states_[1] = joint_state;
  goal_joint_states_[2] = joint_state;

  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 1e4;
  single_joint_limits.jerk_limit = 1e6;
  limits_.clear();
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);

  const double desired_duration = 2.5;
  const double max_duration = desired_duration;
  const double timestep = 0.001;

  TrajectoryGenerator traj_gen(num_dof_, timestep, desired_duration, max_duration, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep, desired_duration, max_duration, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  // Position error
  const double position_tolerance = 1e-4;
  const double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  double timestep_tolerance = 0.1 * timestep;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep,
              timestep_tolerance);
  // Duration
  uint num_waypoint_tolerance = 1;
  uint expected_num_waypoints = 1 + desired_duration / timestep;
  EXPECT_NEAR(uint(output_trajectories_[0].positions.size()), expected_num_waypoints, num_waypoint_tolerance);
}

TEST_F(TrajectoryGenerationTest, DurationExtension)
{
  // The third joint cannot reach in the desired time, so trajectories must be extended

  KinematicState joint_state;
  joint_state.position = -1;
  joint_state.velocity = -0.1;
  joint_state.acceleration = 0;
  current_joint_states_[0] = joint_state;
  current_joint_states_[1] = joint_state;
  current_joint_states_[2] = joint_state;

  // No position change for the first two joints
  joint_state.position = -1;
  joint_state.velocity = 1.9;
  joint_state.acceleration = 0;
  goal_joint_states_[0] = joint_state;
  goal_joint_states_[1] = joint_state;
  // Big position change for the third joint
  joint_state.position = 0;
  goal_joint_states_[2] = joint_state;

  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 1e2;
  single_joint_limits.jerk_limit = 1e4;
  limits_.clear();
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);

  const double desired_duration = 0.1;
  const double max_duration = 5;
  const double timestep = 0.001;

  TrajectoryGenerator traj_gen(num_dof_, timestep, desired_duration, max_duration, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep, desired_duration, max_duration, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  // Position error
  const double position_tolerance = 5e-4;
  const double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  double timestep_tolerance = 0.1 * timestep;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep,
              timestep_tolerance);
  // Duration
  const double expected_duration = 0.865;
  size_t vector_length = output_trajectories_[0].elapsed_times.size() - 1;
  EXPECT_LE(output_trajectories_[0].elapsed_times(vector_length), expected_duration);
}

TEST_F(TrajectoryGenerationTest, PositiveAndNegativeLimits)
{
  // This test encounters negative and positive velocity limits and negative
  // jerk limits

  KinematicState joint_state;
  joint_state.position = -1;
  joint_state.velocity = -0.2;
  joint_state.acceleration = 0;
  current_joint_states_[0] = joint_state;
  joint_state.position = -1;
  joint_state.velocity = 0.1;
  current_joint_states_[1] = joint_state;
  joint_state.position = 1;
  joint_state.velocity = 0.2;
  current_joint_states_[2] = joint_state;

  joint_state.position = -0.9;
  joint_state.velocity = 0.1;
  goal_joint_states_[0] = joint_state;
  joint_state.position = -0.9;
  joint_state.velocity = -0.1;
  goal_joint_states_[1] = joint_state;
  joint_state.position = 0.9;
  joint_state.velocity = 0;
  goal_joint_states_[2] = joint_state;

  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 0.21;
  single_joint_limits.acceleration_limit = 20;
  single_joint_limits.jerk_limit = 10;
  limits_.clear();
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);

  const double timestep = 0.001;
  const double desired_duration = 1800 * timestep;
  const double max_duration = 1800 * timestep;

  TrajectoryGenerator traj_gen(num_dof_, timestep, desired_duration, max_duration, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep, desired_duration, max_duration, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  // Position error
  const double position_tolerance = 1e-4;
  const double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  double timestep_tolerance = 0.1 * timestep;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep,
              timestep_tolerance);
  // Duration
  uint num_waypoint_tolerance = 1;
  uint expected_num_waypoints = 1 + desired_duration / timestep;
  EXPECT_NEAR(uint(output_trajectories_[0].positions.size()), expected_num_waypoints, num_waypoint_tolerance);
}

TEST_F(TrajectoryGenerationTest, TimestepDidNotMatch)
{
  // This test comes from MoveIt. Originally, the final timestep did not match the desired timestep.

  timestep_ = 0.0075;
  desired_duration_ = 0.028322;
  num_dof_ = 1;

  current_joint_states_.resize(num_dof_);
  KinematicState joint_state;
  joint_state.position = 0.00596041;
  joint_state.velocity = -0.176232;
  joint_state.acceleration = -3.06289;
  current_joint_states_[0] = joint_state;

  goal_joint_states_.resize(num_dof_);
  joint_state.position = -0.00121542;
  joint_state.velocity = -0.289615;
  joint_state.acceleration = -2.88021;
  goal_joint_states_[0] = joint_state;

  limits_.resize(num_dof_);
  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 3.15;
  single_joint_limits.acceleration_limit = 5;
  single_joint_limits.jerk_limit = 5000;
  limits_[0] = single_joint_limits;

  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration_, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep_, desired_duration_, max_duration_, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  output_trajectories_.resize(num_dof_);

  EXPECT_EQ(ErrorCodeEnum::NO_ERROR,
            traj_gen.inputChecking(current_joint_states_, goal_joint_states_, limits_, timestep_));
  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  // Position error
  const double position_tolerance = 2e-3;
  const double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  const double timestep_tolerance = 0.0005;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep_,
              timestep_tolerance);
}

TEST_F(TrajectoryGenerationTest, CustomerStreaming)
{
  // A customer-requested streaming test.
  // For simplicity, only Joint 0 is updated
  // This is also a good test of trajectory synchronization in streaming mode.

  timestep_ = 0.001;
  max_duration_ = 100;
  use_streaming_mode_ = true;

  constexpr std::size_t joint_to_update = 0;
  // Position tolerance for each waypoint
  constexpr double waypoint_position_tolerance = 1e-5;
  // Tolerances for the final waypoint
  constexpr double final_position_tolerance = 1e-5;
  constexpr double final_velocity_tolerance = 1e-3;
  constexpr double final_acceleration_tolerance = 1e-2;
  const double min_desired_duration = timestep_;
  // Between iterations, skip this many waypoints.
  // Take next_waypoint from the previous trajectory to start the new trajectory.
  // Minimum is 1.
  constexpr std::size_t next_waypoint = 1;

  current_joint_states_[0].position = 0.9;
  current_joint_states_[1].position = 0.4;
  current_joint_states_[2].position = -1.7;
  goal_joint_states_[0].position = -0.9;
  goal_joint_states_[1].position = -0.9;
  goal_joint_states_[2].position = -0.9;

  Limits limits_per_joint;
  limits_per_joint.velocity_limit = 2;
  limits_per_joint.acceleration_limit = 2;
  limits_per_joint.jerk_limit = 2;
  limits_ = { limits_per_joint, limits_per_joint, limits_per_joint };

  // This is a best-case estimate, assuming the robot is already at maximum velocity
  double desired_duration =
      fabs(current_joint_states_[joint_to_update].position - goal_joint_states_[joint_to_update].position) /
      limits_[joint_to_update].velocity_limit;
  // But, don't ask for a duration that is shorter than one timestep
  desired_duration_ = std::max(desired_duration_, min_desired_duration);

  // Generate initial trajectory
  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration, max_duration_, current_joint_states_,
                               goal_joint_states_, limits_, waypoint_position_tolerance, use_streaming_mode_);
  traj_gen.reset(timestep_, desired_duration, max_duration_, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  ErrorCodeEnum error_code = traj_gen.generateTrajectories(&output_trajectories_);
  EXPECT_EQ(error_code, ErrorCodeEnum::NO_ERROR);

  double position_error = std::numeric_limits<double>::max();
  double velocity_error = std::numeric_limits<double>::max();
  double acceleration_error = std::numeric_limits<double>::max();

  while (fabs(position_error) > final_position_tolerance || fabs(velocity_error) > final_velocity_tolerance ||
         fabs(acceleration_error) > final_acceleration_tolerance)
  {
    traj_gen.reset(timestep_, desired_duration, max_duration_, current_joint_states_, goal_joint_states_, limits_,
                   waypoint_position_tolerance, use_streaming_mode_);
    error_code = traj_gen.generateTrajectories(&output_trajectories_);
    EXPECT_EQ(error_code, ErrorCodeEnum::NO_ERROR);
    // Get a new seed state for next trajectory generation
    if ((std::size_t)output_trajectories_.at(joint_to_update).positions.size() > next_waypoint)
    {
      current_joint_states_[joint_to_update].position =
          output_trajectories_.at(joint_to_update).positions[next_waypoint];
      current_joint_states_[joint_to_update].velocity =
          output_trajectories_.at(joint_to_update).velocities[next_waypoint];
      current_joint_states_[joint_to_update].acceleration =
          output_trajectories_.at(joint_to_update).accelerations[next_waypoint];
    }

    position_error = current_joint_states_[joint_to_update].position - goal_joint_states_.at(joint_to_update).position;
    velocity_error = current_joint_states_[joint_to_update].velocity - goal_joint_states_.at(joint_to_update).velocity;
    acceleration_error =
        current_joint_states_[joint_to_update].acceleration - goal_joint_states_.at(joint_to_update).acceleration;

    // Shorten the desired duration as we get closer to goal
    desired_duration -= timestep_;
    // But, don't ask for a duration that is shorter than the minimum
    desired_duration = std::max(desired_duration, min_desired_duration);
  }

  // If the test gets here, it passed.
}

TEST_F(TrajectoryGenerationTest, StreamingTooFewTimesteps)
{
  // An error should be thrown if streaming mode is enabled with a desired duration < kMinNumTimesteps

  use_streaming_mode_ = true;
  desired_duration_ = 9 * timestep_;

  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration_, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep_, desired_duration_, max_duration_, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  EXPECT_EQ(ErrorCodeEnum::LESS_THAN_TEN_TIMESTEPS_FOR_STREAMING_MODE,
            traj_gen.inputChecking(current_joint_states_, goal_joint_states_, limits_, timestep_));
}

TEST_F(TrajectoryGenerationTest, SingleJointOscillation)
{
  // This test comes from MoveIt. Originally, this joint's trajectory oscillated
  // Fixed by setting a shorter desired_duration_

  timestep_ = 0.0075;
  desired_duration_ = 0.1;
  num_dof_ = 1;

  current_joint_states_.resize(num_dof_);
  KinematicState joint_state;
  joint_state.position = -0.00397532;
  joint_state.velocity = -0.169886;
  joint_state.acceleration = -0.84943;
  current_joint_states_[0] = joint_state;

  goal_joint_states_.resize(num_dof_);
  joint_state.position = -0.025211;
  joint_state.velocity = -0.253157;
  joint_state.acceleration = -0.87268;
  goal_joint_states_[0] = joint_state;

  limits_.resize(num_dof_);
  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 3.15;
  single_joint_limits.acceleration_limit = 5;
  single_joint_limits.jerk_limit = 5000;
  limits_[0] = single_joint_limits;

  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration_, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep_, desired_duration_, max_duration_, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  output_trajectories_.resize(num_dof_);

  EXPECT_EQ(ErrorCodeEnum::NO_ERROR,
            traj_gen.inputChecking(current_joint_states_, goal_joint_states_, limits_, timestep_));
  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  // Position error
  const double position_tolerance = 2e-3;
  const double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  const double timestep_tolerance = 0.0005;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep_,
              timestep_tolerance);
}

TEST_F(TrajectoryGenerationTest, VelocityLimit)
{
  // Velocity limit is hit

  const double max_duration = 20;

  KinematicState joint_state;

  current_joint_states_[0] = joint_state;
  current_joint_states_[1] = joint_state;
  current_joint_states_[2] = joint_state;

  joint_state.position = -0.995;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  goal_joint_states_[0] = joint_state;
  goal_joint_states_[1] = joint_state;
  goal_joint_states_[2] = joint_state;

  // Maximums prior to trackjoint: velocity: 1.86513, accel: 5.74272, jerk: 54.4166
  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 1;  // Limit velocity from 1.86513 to 1
  single_joint_limits.acceleration_limit = 20;
  single_joint_limits.jerk_limit = 2000;
  limits_.clear();
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);

  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep_, desired_duration_, max_duration, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  traj_gen.generateTrajectories(&output_trajectories_);

  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  verifyVelAccelJerkLimits(output_trajectories_, limits_);

  // Position error
  double position_tolerance = 1e-4;
  double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  double timestep_tolerance = 0.1 * timestep_;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep_,
              timestep_tolerance);
  // Duration
  size_t vector_length = output_trajectories_[0].elapsed_times.size() - 1;
  EXPECT_LE(output_trajectories_[0].elapsed_times(vector_length), desired_duration_);
}

TEST_F(TrajectoryGenerationTest, AccelerationLimit)
{
  // Acceleration limit is hit

  const double max_duration = 20;

  KinematicState joint_state;

  current_joint_states_[0] = joint_state;
  current_joint_states_[1] = joint_state;
  current_joint_states_[2] = joint_state;

  joint_state.position = -0.995;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  goal_joint_states_[0] = joint_state;
  goal_joint_states_[1] = joint_state;
  goal_joint_states_[2] = joint_state;

  // Maximums prior to trackjoint: velocity: 1.86513, accel: 5.74272, jerk: 54.4166
  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 5;  // Limit accel from 5.74272 to 5
  single_joint_limits.jerk_limit = 2000;
  limits_.clear();
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);

  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep_, desired_duration_, max_duration, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  traj_gen.generateTrajectories(&output_trajectories_);

  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  verifyVelAccelJerkLimits(output_trajectories_, limits_);

  // Position error
  double position_tolerance = 1e-4;
  double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  double timestep_tolerance = 0.1 * timestep_;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep_,
              timestep_tolerance);
  // Duration
  size_t vector_length = output_trajectories_[0].elapsed_times.size() - 1;
  EXPECT_LE(output_trajectories_[0].elapsed_times(vector_length), desired_duration_);
}

TEST_F(TrajectoryGenerationTest, JerkLimit)
{
  // Jerk limit is hit

  const double max_duration = 20;

  KinematicState joint_state;

  current_joint_states_[0] = joint_state;
  current_joint_states_[1] = joint_state;
  current_joint_states_[2] = joint_state;

  joint_state.position = -0.995;
  joint_state.velocity = 0;
  joint_state.acceleration = 0;
  goal_joint_states_[0] = joint_state;
  goal_joint_states_[1] = joint_state;
  goal_joint_states_[2] = joint_state;

  // Maximums prior to trackjoint: velocity: 1.86513, accel: 5.74272, jerk: 54.4166
  Limits single_joint_limits;
  single_joint_limits.velocity_limit = 2;
  single_joint_limits.acceleration_limit = 20;
  single_joint_limits.jerk_limit = 50;  // Limit jerk from 54.4166 to 50
  limits_.clear();
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);
  limits_.push_back(single_joint_limits);

  TrajectoryGenerator traj_gen(num_dof_, timestep_, desired_duration_, max_duration, current_joint_states_,
                               goal_joint_states_, limits_, position_tolerance_, use_streaming_mode_);
  traj_gen.reset(timestep_, desired_duration_, max_duration, current_joint_states_, goal_joint_states_, limits_,
                 position_tolerance_, use_streaming_mode_);
  traj_gen.generateTrajectories(&output_trajectories_);

  EXPECT_EQ(ErrorCodeEnum::NO_ERROR, traj_gen.generateTrajectories(&output_trajectories_));

  verifyVelAccelJerkLimits(output_trajectories_, limits_);

  // Position error
  double position_tolerance = 1e-4;
  double position_error = calculatePositionAccuracy(goal_joint_states_, output_trajectories_);
  EXPECT_LT(position_error, position_tolerance);
  // Timestep
  double timestep_tolerance = 0.1 * timestep_;
  EXPECT_NEAR(output_trajectories_[0].elapsed_times[1] - output_trajectories_[0].elapsed_times[0], timestep_,
              timestep_tolerance);
  // Duration
  size_t vector_length = output_trajectories_[0].elapsed_times.size() - 1;
  EXPECT_LE(output_trajectories_[0].elapsed_times(vector_length), desired_duration_);
}

}  // namespace trackjoint

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  return result;
}
