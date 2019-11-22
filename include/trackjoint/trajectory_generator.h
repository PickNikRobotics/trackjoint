/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: Generate jerk-limited trajectories for n robot joints.
*/

#pragma once

// TrackJoint
#include <trackjoint/error_codes.h>
#include <trackjoint/kinematic_state.h>
#include <trackjoint/limits.h>
#include <trackjoint/single_joint_generator.h>

// C++
#include <memory>  // shared_ptr
#include <Eigen/Geometry>
#include <vector>

// Testing
#include <gtest/gtest.h>


namespace trackjoint
{

/**
 * \brief A point of the output trajectory in Cartesian space, with time parameterization
 */
struct TrajectoryWaypoint
{
  // Translational and angular state of 6dof Cartesian pose for position, velocity, and acceleration
  KinematicState state;

  // Time elapsed since start of trajectory in seconds
  double elapsed_time;
};

class TrajectoryGenerator
{
public:
  /** \brief Constructor */
  TrajectoryGenerator(uint num_dof, double timestep,
	double desired_duration, double max_duration,
	std::vector<KinematicState> &current_joint_states,
	std::vector<KinematicState> &goal_joint_states,
	std::vector<Limits> &limits);

  /** \brief Generate and return trajectories for every joint*/
  void GenerateTrajectories(std::vector<std::vector<TrajectoryWaypoint>> &output_trajectories);

  /** \brief Save generated trajectory to a .csv file */
  void SaveTrajectoriesToFile(const std::vector<std::vector<TrajectoryWaypoint>> &output_trajectories,
    const std::string &base_filepath) const;

protected:
  uint num_dof_;
  double desired_duration_;
  uint error_code_ = ErrorCodeEnum::NO_ERROR;
  std::vector<trackjoint::SingleJointGenerator> single_joint_generators_;
  double upsampled_timestep_;

  // --------------------------------------------------------
  // Any test that requires access to protected variables should go here
  FRIEND_TEST(TestTrajectoryGenerator, TestNameOfClass);
};  // end class TrajectoryGenerator

// Create std pointers for this class
typedef std::shared_ptr<TrajectoryGenerator> TrajectoryGeneratorPtr;
typedef std::shared_ptr<const TrajectoryGenerator> TrajectoryGeneratorConstPtr;

}  // namespace trackjoint
