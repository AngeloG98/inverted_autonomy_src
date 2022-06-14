cmake_minimum_required(VERSION 3.0.2)
project(flight_common)

# add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  flight_msgs
  mavros
  mavros_extras
  mavros_msgs
  mavlink
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES flight_common
 CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/control_command.cpp
  src/state_estimate.cpp
  src/trajectory_point.cpp
  src/math_common.cpp
  src/geometry_eigen_conversions.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

