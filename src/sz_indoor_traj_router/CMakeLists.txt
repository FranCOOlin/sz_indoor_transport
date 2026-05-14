cmake_minimum_required(VERSION 3.0.2)
project(sz_indoor_traj_router)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sz_indoor_controller
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sz_indoor_controller
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(trajectory_node src/trajectory.cpp)

add_dependencies(trajectory_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(trajectory_node
  ${catkin_LIBRARIES}
)
