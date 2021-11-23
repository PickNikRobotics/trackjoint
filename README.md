# trackjoint

Description: A trajectory-smoothing library

<img src="https://picknik.ai/assets/images/logo.jpg" width="100">

Developed by Andy Zelenak at [PickNik Consulting](http://picknik.ai/)

TODO(andyze): fix Travis badge:
[![Build Status](https://travis-ci.com/PickNikRobotics/trackjoint.svg?token=o9hPQnr2kShM9ckDs6J8&branch=master)](https://travis-ci.com/PickNikRobotics/trackjoint)

## What does this package do?

TrackJoint takes an input state consisting of (position, velocity, acceleration) and a target state. It fits a spline from input to target state, then smooths the spline until each waypoint obeys the given (velocity, acceleration, and jerk) kinematic limits. In other words, it is great at generating a trajectory from State A to State B.

TrackJoint also has a streaming mode which is intended for the smoothing of realtime commands such as from a joystick or visual servoing.

## What doesn't this package do?

TrackJoint is not intended to generate a trajectory passing through multiple vias or waypoints.

## Install

### Build from Source

1. Clone this package into a colcon workspace

2. `colcon build`

## Run

1. `ros2 run trackjoint simple_example`  (for example)

## Code API

> Note: this package has not been released yet

See [the Doxygen documentation](http://docs.ros.org/melodic/api/trackjoint/html/anotated.html)

## Testing

    colc_build_test

    colc_test_pkg trackjoint

## Formatting

Install pre-commit:  `pre-commit install`

It will automatically format the repo whenever you make a commit.

## Hints for Tuning

Usually a robot manufacturer provides joint velocity limits. It can be difficult to set joint acceleration or jerk limits when the manufacturer does not provide them. Here are some hints to help choose reasonable values:

* The jerk limit is usually greater than the acceleration limit, which is greater than the velocity limit: jerk_limit > acceleration_limit > velocity_limit.

* The jerk limit should be less than (acceleration_limit / controller_timestep). Otherwise, acceleration could jump to maximum in one timestep and there would be no reason for a jerk limit.

* For tuning the acceleration limit, think about how long it should take the robot to accelerate to maximum velocity in your application. For example, if it takes approximately 0.5 seconds to accelerate to full speed, an estimate is:  acceleration_limit = velocity_limit / 0.5.

## Third Party Open Source Software Licenses

The following libraries are used in TrackJoint:

 - [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page#License)
 - [googletest](https://github.com/google/googletest/blob/master/googletest/LICENSE)
 - [Standard Template Library "algorithm"](https://github.com/google/libcxx/blob/master/LICENSE.TXT)
