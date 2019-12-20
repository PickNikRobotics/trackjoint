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

## Third Party Open Source Software Licenses

The following libraries are used in TrackJoint:

 - [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page#License)
 - [googletest](https://github.com/google/googletest/blob/master/googletest/LICENSE)
 - [Standard Template Library "algorithm"](https://github.com/google/libcxx/blob/master/LICENSE.TXT)
