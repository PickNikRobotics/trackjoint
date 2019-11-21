/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
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
  TrajectoryGenerator(const uint num_dof, const double timestep,
	const double desired_duration, const double max_duration,
	std::vector<KinematicState> &current_joint_states,
	std::vector<KinematicState> &goal_joint_states,
	std::vector<Limits> &limits, const double velocity_tolerance,
	const double acceleration_tolerance, const double jerk_tolerance);

  /** \brief Generate and return trajectories for every joint*/
  void GenerateTrajectories(std::vector<std::vector<TrajectoryWaypoint>> &output_trajectories);

  /** \brief Save generated trajectory to a .csv file */
  void SaveTrajectoriesToFile(std::vector<std::vector<TrajectoryWaypoint>> &output_trajectories,
    const std::string base_filepath);

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
