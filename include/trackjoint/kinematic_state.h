/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

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
  double position;  // radians
  double velocity;
  double acceleration;

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
