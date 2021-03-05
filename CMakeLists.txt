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

set(THIS_PACKAGE_INCLUDE_DEPENDS)

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
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

set(SIMPLE_EXAMPLE simple_example)
add_executable(${SIMPLE_EXAMPLE} src/simple_example.cpp)
ament_target_dependencies(${SIMPLE_EXAMPLE} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${SIMPLE_EXAMPLE} ${LIBRARY_NAME})

set(STREAMING_EXAMPLE streaming_example)
add_executable(${STREAMING_EXAMPLE} src/streaming_example.cpp)
ament_target_dependencies(${STREAMING_EXAMPLE} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${STREAMING_EXAMPLE} ${LIBRARY_NAME})

set(THREE_DOF_EXAMPLES three_dof_examples)
add_executable(${THREE_DOF_EXAMPLES} src/three_dof_examples.cpp)
ament_target_dependencies(${THREE_DOF_EXAMPLES} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${THREE_DOF_EXAMPLES} ${LIBRARY_NAME})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME} ${SIMPLE_EXAMPLE} ${STREAMING_EXAMPLE} ${THREE_DOF_EXAMPLES}
  ARCHIVE DESTINATION
    lib
  LIBRARY DESTINATION
    lib
  RUNTIME DESTINATION
    lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

# TODO(andyz): update this for ROS2

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
