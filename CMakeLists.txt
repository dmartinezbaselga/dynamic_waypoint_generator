cmake_minimum_required(VERSION 2.8.3)
project(dynamic_waypoint_generator)

add_definitions(-DBOOST_LOG_DYN_LINK=1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs genmsg roslib roslaunch std_srvs geometry_msgs sensor_msgs laser_geometry message_generation rviz tf)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -L/usr/include -Wall -Wextra -Weffc++ -ansi -pedantic -g -rdynamic -w -frounding-math")


add_definitions("-std=c++14")

#catkin_python_setup()

#add_message_files(FILES)
# add_service_files(
#   FILES
# )
# generate_messages(DEPENDENCIES std_msgs geometry_msgs)

#set(dynamic_waypoint_generator_LINK_LIBRARIES "${Boost_LIBRARIES} CGAL dislin gmp mpfr Xm ${catkin_LIBRARIES}")

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs tf laser_geometry message_runtime rviz
)

# set(catkin_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS} dynamic_waypoint_generator_generate_messages_cpp)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})
#link_directories(include/dynamic_waypoint_generator/)


add_executable(global_planner_dovs src/global_planner_dovs.cpp)
target_link_libraries(global_planner_dovs ${catkin_LIBRARIES})
