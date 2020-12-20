/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: John Morris
   Desc: Trajectory generator configuration.
*/

#pragma once

#include "trackjoint/limits.h"

namespace trackjoint
{
/**
 * \brief Trajectory generator configuration.
 */
struct Configuration
{
  Configuration(double timestep, double max_duration, const Limits& limits, const double position_tolerance)
    : timestep(timestep)
    , max_duration(max_duration)
    , limits(limits)
    , position_tolerance(position_tolerance)
  {
  }

  Configuration()
  {
  }

  double timestep;
  double max_duration;
  Limits limits;
  double position_tolerance;
};
}  // namespace trackjoint
