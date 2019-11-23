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

#include <Eigen/Dense>

namespace trackjoint {
/**
 * \brief Vectors of position, velocity, acceleration, and time
 */
struct JointTrajectory {
  Eigen::VectorXd positions;  // rad
  Eigen::VectorXd velocities;  // rad/s
  Eigen::VectorXd accelerations;  // rad/s^2
  Eigen::VectorXd jerks;  // rad/s^3
  Eigen::VectorXd elapsed_times;  // seconds
};
}  // namespace trackjoint
