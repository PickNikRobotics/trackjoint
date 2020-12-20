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
  roscpp
  roslib
  roslint
  rospy
)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    roslib
    rospy
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(LIBRARY_NAME trackjoint)
add_library(${LIBRARY_NAME}
  src/trajectory_generator.cpp
  src/single_joint_generator.cpp
  src/utilities.cpp
)
target_link_libraries(
  ${LIBRARY_NAME}
  ${catkin_LIBRARIES}
)

# A minimal, single-trajectory example
add_executable(
  ${PROJECT_NAME}_simple_example
  src/simple_example.cpp
)
# Rename C++ executable without namespace
set_target_properties(
  ${PROJECT_NAME}_simple_example
  PROPERTIES
    OUTPUT_NAME simple_example PREFIX ""
)
target_link_libraries(
  ${PROJECT_NAME}_simple_example
  ${LIBRARY_NAME}
  ${catkin_LIBRARIES}
)

add_executable(
  ${PROJECT_NAME}_three_dof_examples
  src/three_dof_examples.cpp
)

set_target_properties(
  ${PROJECT_NAME}_three_dof_examples
  PROPERTIES
    OUTPUT_NAME three_dof_examples PREFIX ""
)

target_link_libraries(
  ${PROJECT_NAME}_three_dof_examples
  ${LIBRARY_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME} ${PROJECT_NAME}_simple_example
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(trajectory_generation_test
    test/trajectory_generation_test.test
    test/trajectory_generation_test.cpp
  )
  target_link_libraries(trajectory_generation_test
    ${LIBRARY_NAME}
    ${catkin_LIBRARIES}
  )

  add_rostest_gtest(single_joint_generator_test
    test/single_joint_generator_test.test
    test/single_joint_generator_test.cpp
  )
  target_link_libraries(single_joint_generator_test
    ${LIBRARY_NAME}
    ${catkin_LIBRARIES}
  )

endif()

## Test for correct C++ source code
roslint_cpp()
