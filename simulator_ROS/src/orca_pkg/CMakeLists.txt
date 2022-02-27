cmake_minimum_required(VERSION 2.8.3)
project(orca_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(include
        include/orca
        include/orca_pkg
        ${catkin_INCLUDE_DIRS}
        )
add_library(${PROJECT_NAME} src/orca/qrotor_control_orca_impl.cpp
        src/orca/Agent.cpp
        src/orca/KdTree.cpp
        src/orca/ORCA_API.cpp
        src/orca/RVOSimulator.cpp
        src/orca/orca.cpp)


add_executable(test_orca src/act_orca_test.cpp)
target_link_libraries(test_orca ${PROJECT_NAME})
target_link_libraries(test_orca ${catkin_LIBRARIES})
