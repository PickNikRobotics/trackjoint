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

# No ROS dependencies yet
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
add_library(${LIBRARY_NAME} SHARED
  src/trajectory_generator.cpp
  src/single_joint_generator.cpp
  src/utilities.cpp
)
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${LIBRARY_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${LIBRARY_NAME}
  ${Boost_FILESYSTEM_LIBRARY}
)

add_executable(simple_example src/simple_example.cpp)
ament_target_dependencies(simple_example ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(simple_example ${LIBRARY_NAME})

add_executable(streaming_example src/streaming_example.cpp)
ament_target_dependencies(streaming_example ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(streaming_example ${LIBRARY_NAME})

add_executable(three_dof_examples src/three_dof_examples.cpp)
ament_target_dependencies(three_dof_examples ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(three_dof_examples ${LIBRARY_NAME})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${LIBRARY_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
