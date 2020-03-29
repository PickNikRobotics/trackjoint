/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: Describe the kinematic state at an instant.
*/

#pragma once

#include <iostream>

namespace trackjoint
{
/**
 * \brief Joint position, velocity, and acceleration
 */
class KinematicState
{
public:
  // State at this waypoint in a global, inertial reference frame
  double position = 0.0;  // radians
  double velocity = 0.0;
  double acceleration = 0.0;

  /** \brief Print this state
   */
  void print()
  {
    std::cout << "Position:" << std::endl;
    std::cout << this->position << std::endl << std::endl;

    std::cout << "Velocity:" << std::endl;
    std::cout << this->velocity << std::endl << std::endl;

    std::cout << "Acceleration:" << std::endl;
    std::cout << this->acceleration << std::endl;
  }
};
}  // namespace trackjoint
