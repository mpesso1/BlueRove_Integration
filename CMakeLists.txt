cmake_minimum_required(VERSION 3.0.2)
project(blue_rov_custom_integration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

add_service_files (
  FILES
  update_waypoint.srv
)

generate_messages(
   DEPENDENCIES
   geometry_msgs   nav_msgs   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blue_rov_custom_integration
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# pid_lib
add_library(pid_lib lib/pid/ros_pid.cpp lib/pid/ros_pid.h)
target_include_directories(pid_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/lib/pid")

# pp_lib
add_library(pp_lib lib/Path_Planner/goPath.cpp lib/Path_Planner/goPath.h)
target_include_directories(pp_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/lib/Path_Planner")

# pp_node
add_executable(pp_node src/Path_Planner/pp_goPath.cpp)

# Controller_node
add_executable(Controller_node src/pid/pp_pid.cpp)
#add_executable(tester_node src/test_subscriber.cpp)

target_link_libraries(Controller_node ${catkin_LIBRARIES} pid_lib)
target_link_libraries(pp_node ${catkin_LIBRARIES} pp_lib)
#target_link_libraries(tester_node ${catkin_LIBRARIES})



