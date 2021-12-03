cmake_minimum_required(VERSION 3.0.2)
project(blue_rov_custom_integration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blue_rov_custom_integration
   CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pid_lib lib/pid/ros_pid.cpp lib/pid/ros_pid.h)
target_include_directories(pid_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/lib/pid")

add_library(pp_lib lib/Path_Planner/goPath.cpp lib/Path_Planner/goPath.h)
target_include_directories(pp_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/lib/Path_Planner")

add_executable(pp_node src/Path_Planner/pp_goPath.cpp)
add_executable(Controller_node src/pid/pp_pid.cpp)
#add_executable(tester_node src/test_subscriber.cpp)

target_link_libraries(Controller_node ${catkin_LIBRARIES} pid_lib)
target_link_libraries(pp_node ${catkin_LIBRARIES} pp_lib)
#target_link_libraries(tester_node ${catkin_LIBRARIES})



