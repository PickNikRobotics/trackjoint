cmake_minimum_required(VERSION 2.8.3)
project(trackjoint)

# C++ 14
add_compile_options(-std=c++14)

# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Boost
    EIGEN3
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  SYSTEM
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library with project namespace to avoid naming collision
add_library(
  ${PROJECT_NAME}
  src/track_joint.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Declare a C++ executable
add_executable(
  ${PROJECT_NAME}_run_example
  src/run_example.cpp
)

# Rename C++ executable without namespace
set_target_properties(
  ${PROJECT_NAME}_run_example
  PROPERTIES
    OUTPUT_NAME run_example PREFIX ""
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}_run_example
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Testing ##
#############

# TODO(andyz): add gtest

## Test for correct C++ source code
roslint_cpp()
