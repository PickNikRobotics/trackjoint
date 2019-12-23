# trackjoint

Description: A trajectory-smoothing library

<img src="https://picknik.ai/assets/images/logo.jpg" width="100">

Developed by Andy Zelenak at [PickNik Consulting](http://picknik.ai/)

TODO(andyze): fix Travis badge:
[![Build Status](https://travis-ci.com/PickNikRobotics/trackjoint.svg?token=o9hPQnr2kShM9ckDs6J8&branch=master)](https://travis-ci.com/PickNikRobotics/trackjoint)

## Install

### Build from Source

1. Clone this package into a catkin workspace

2. `catkin build`

## Run

1. rosrun trackjoint run_example

(You don't need to start `roscore` because the executable doesn't use ROS)

## Code API

> Note: this package has not been released yet

See [the Doxygen documentation](http://docs.ros.org/melodic/api/trackjoint/html/anotated.html)

## Testing

    catkin run_tests --no-deps trackjoint -i

## Formatting

Use `clang_this_directory_google`

## Hints for Tuning

Usually a robot manufacturer provides joint velocity limits. It can be difficult to set joint acceleration or jerk limits when the manufacturer does not provide them. Here are some hints to help choose reasonable values:

* The jerk limit is usually greater than the acceleration limit, which is greater than the velocity limit: jerk_limit > acceleration_limit > velocity_limit.

* It is not unusual to have jerk limits that seem very large, in the range of 5,000-10,000 rad/s^3.

* For tuning the acceleration limit, think about how long it should take the robot to accelerate to maximum velocity in your application. For example, if it should take approximately 0.5 seconds to accelerate to full speed, an estimate is:  acceleration_limit = velocity_limit / 0.5.
