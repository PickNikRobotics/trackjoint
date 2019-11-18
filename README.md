# trackjoint

Description: A trajectory-smoothing library

<img src="https://picknik.ai/assets/images/logo.jpg" width="100">

Developed by Andy Zelenak at [PickNik Consulting](http://picknik.ai/)

TODO(andyze): fix Travis badge:
[![Build Status](https://travis-ci.com/PickNikRobotics/trackjoint.svg?token=o9hPQnr2kShM9ckDs6J8&branch=master)](https://travis-ci.com/PickNikRobotics/trackjoint)

## Install

### Ubuntu Debian

> Note: this package has not been released yet

    sudo apt-get install ros-${ROS_DISTRO}-trackjoint

### Build from Source

1. Install [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) if you are running on Ubuntu 16.04 or [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) for Ubuntu 18.04. This package primerily targets melodic

1. Install the following build tools:

        sudo apt-get install python-wstool python-catkin-tools

1. Re-use or create a catkin workspace:

        mkdir -p ~/ws_catkin/
        cd ~/ws_catkin/

1. Download the required repositories and install any dependencies:

        git clone git@github.com:PickNikRobotics/trackjoint.git
        wstool init src
        wstool merge -t src trackjoint/trackjoint.rosinstall
        wstool update -t src
        rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}

1. Configure and build the workspace:

        catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin build

1. Source the workspace.

        source devel/setup.bash

## Developers: Quick update code repositories

To make sure you have the latest repos:

    cd ~/ws_catkin/src/trackjoint
    git checkout master
    git pull origin master
    cd ..
    wstool merge trackjoint/trackjoint.rosinstall
    wstool update
    rosdep install --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}

## Run

Run run_example
```
roslaunch trackjoint run_example.launch
```

### Run with Debuging

Run run_example with GDB
```
roslaunch trackjoint run_example.launch debug:=true
```

Run run_example with Callgrind
```
roslaunch trackjoint run_example.launch callgrind:=true
```

Run run_example with Valgrind
```
roslaunch trackjoint run_example.launch valgrind:=true
```

## Run Inside Docker

### Prerequisite

You must have a private rsa key `~/.ssh/id_rsa` that is not password protected and is attached to your Github/Bitbucket/Gerrit accounts.
You must also have a working installation of `docker`.

1. Navigate to `$CATKIN_WS/src/trackjoint/.docker`. You should see the `Dockerfile` recipe in the directory.

1. Build the docker image

        cd $CATKIN_WS/src/trackjoint/.docker
        cp ~/.ssh/id_rsa id_rsa && docker build -t trackjoint:melodic-source .; rm id_rsa

1. Run the docker image

    * Without the gui

            docker run -it --rm trackjoint:melodic-source /bin/bash

    * With the gui (tested with Ubuntu native and a Ubuntu VM)

            . ./gui-docker -it --rm trackjoint:melodic-source /bin/bash

## Code API

> Note: this package has not been released yet

See [the Doxygen documentation](http://docs.ros.org/melodic/api/trackjoint/html/anotated.html)

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    roscd trackjoint
    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    catkin lint -W2 --rosdistro ${ROS_DISTRO}

Use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/) to run tests.

    catkin run_tests

To run tests for just one package:

    catkin run_tests --make-args tests -- trackjoint

To view test results:

    cd $CATKIN_WS
    catkin_test_results --all

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

## Using ccache

ccache is a useful tool to speed up compilation times with GCC or any other sufficiently similar compiler.

To install ccache on Linux:

    sudo apt-get install ccache

To use ccache add it to your ``PATH`` in front of your regular compiler. It is recommended that you add this line to your `.bashrc`:

    export PATH=/usr/lib/ccache:$PATH
