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
  Configuration(double timestep, double max_duration, const Limits& limits, const double position_tolerance,
                bool use_streaming_mode)
    : timestep(timestep)
    , max_duration(max_duration)
    , limits(limits)
    , position_tolerance(position_tolerance)
    , use_streaming_mode(use_streaming_mode)
  {
  }

  Configuration()
  {
  }

  double timestep;
  double max_duration;
  Limits limits;
  double position_tolerance;
  // If streaming mode is enabled, trajectories are clipped at
  // kNumWaypointsThreshold so the algorithm runs very quickly.
  //
  // Streaming mode is intended for realtime streaming applications.
  //
  // There could be even fewer waypoints than that if the goal is very
  // close or the algorithm only finds a few waypoints successfully.
  //
  // In streaming mode, trajectory duration is not extended until it
  // successfully reaches the goal.
  bool use_streaming_mode;
};
}  // namespace trackjoint
