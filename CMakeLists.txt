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

# Declare a C++ library with project namespace to avoid naming collision
set(LIBRARY_NAME trackjoint)
add_library(${LIBRARY_NAME} SHARED
  src/trajectory_generator.cpp
  src/single_joint_generator.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  ${LIBRARY_NAME}
  ${catkin_LIBRARIES}
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
  ${LIBRARY_NAME}
  ${catkin_LIBRARIES}
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

  add_rostest_gtest(helper_function_test
    test/helper_function_test.test
    test/helper_function_test.cpp
  )
  target_link_libraries(helper_function_test
    ${LIBRARY_NAME}
    ${catkin_LIBRARIES}
  )

endif()

## Test for correct C++ source code
roslint_cpp()
