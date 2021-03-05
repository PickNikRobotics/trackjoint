cmake_minimum_required(VERSION 3.5)
project(trackjoint)

# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

find_package(ament_cmake REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Disable ROS stuff for this initial port to ROS2
#find_package(ament_cmake REQUIRED COMPONENTS
#  roscpp
#  roslib
#  roslint
#  rospy
#)

set(THIS_PACKAGE_INCLUDE_DEPENDS)

# catkin_package(
#   CATKIN_DEPENDS
#     roscpp
#     roslib
#     rospy
#   INCLUDE_DIRS
#     include
#   LIBRARIES
#     ${PROJECT_NAME}
#   DEPENDS
#     EIGEN3
# )

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)

include_directories(
  include
)

set(LIBRARY_NAME trackjoint)
add_library(${LIBRARY_NAME}
  src/trajectory_generator.cpp
  src/single_joint_generator.cpp
  src/utilities.cpp
)
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${LIBRARY_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${LIBRARY_NAME}
  ${Boost_FILESYSTEM_LIBRARY}
)

# Just compile one simple example in ament, for now

#add_executable(
#  ${PROJECT_NAME}_simple_example
#  src/simple_example.cpp
#)
## Rename C++ executable without namespace
#set_target_properties(
#  ${PROJECT_NAME}_simple_example
#  PROPERTIES
#    OUTPUT_NAME simple_example PREFIX ""
#)
#target_link_libraries(
#  ${PROJECT_NAME}_simple_example
#  ${LIBRARY_NAME}
#  ${catkin_LIBRARIES}
#)

set(SIMPLE_EXAMPLE_NAME simple_example)
add_executable(${SIMPLE_EXAMPLE_NAME} src/simple_example.cpp)
ament_target_dependencies(${SIMPLE_EXAMPLE_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${SIMPLE_EXAMPLE_NAME} ${LIBRARY_NAME})

## A streaming example (update within a loop)
#add_executable(
#  ${PROJECT_NAME}_streaming_example
#  src/streaming_example.cpp
#)
## Rename C++ executable without namespace
#set_target_properties(
#  ${PROJECT_NAME}_streaming_example
#  PROPERTIES
#    OUTPUT_NAME streaming_example PREFIX ""
#)
#target_link_libraries(
#  ${PROJECT_NAME}_streaming_example
#  ${LIBRARY_NAME}
#  ${catkin_LIBRARIES}
#)

#add_executable(
#  ${PROJECT_NAME}_three_dof_examples
#  src/three_dof_examples.cpp
#)
#set_target_properties(
#  ${PROJECT_NAME}_three_dof_examples
#  PROPERTIES
#    OUTPUT_NAME three_dof_examples PREFIX ""
#)
#target_link_libraries(
#  ${PROJECT_NAME}_three_dof_examples
#  ${LIBRARY_NAME}
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME} ${SIMPLE_EXAMPLE_NAME} 
    #${PROJECT_NAME}_streaming_example
  ARCHIVE DESTINATION
    lib
  LIBRARY DESTINATION
    lib
  RUNTIME DESTINATION
    bin
)

## Mark cpp header files for installation
#install(
#  DIRECTORY
#    include/${PROJECT_NAME}/
#  DESTINATION
#    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

#############
## Testing ##
#############

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#
#  add_rostest_gtest(trajectory_generation_test
#    test/trajectory_generation_test.test
#    test/trajectory_generation_test.cpp
#  )
#  target_link_libraries(trajectory_generation_test
#    ${LIBRARY_NAME}
#    ${catkin_LIBRARIES}
#  )
#
#endif()

# Install include, launch, config directories
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
