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

## Running CI tests locally

It's also possible to run the CI script locally, without Travis.  You must also have a working installation of `docker`.

1. First clone the repo you want to test:

        cd /tmp/travis   # any working directory will do
        git clone https://github.com/PickNikRobotics/trackjoint.git
        cd trackjoint
        git clone -q --depth=1 https://github.com/ros-planning/moveit_ci.git .moveit_ci

1. Manually define the variables, Travis would otherwise define for you. These are required:

        export TRAVIS_BRANCH="$(git rev-parse --abbrev-ref HEAD)"
        export ROS_DISTRO=melodic
        export ROS_REPO=ros
        export CC=gcc
        export CXX=g++

1. Set test variables (see `.travis.yaml` for other options):

        export UPSTREAM_WORKSPACE=trackjoint.rosinstall
        export TEST=clang-format

1. Start the script

        .moveit_ci/travis.sh

It's also possible to run the script without using docker. To this end, issue the following additional commands:

    export IN_DOCKER=1               # pretend running docker
    export CI_SOURCE_PATH=$PWD       # repository location in, i.e. /tmp/travis/trackjoint
    export ROS_WS=/tmp/ros_ws        # define a new ROS workspace location
    mkdir $ROS_WS                    # and create it
    .moveit_ci/travis.sh

## Formatting

Use `clang_this_directory_google`
