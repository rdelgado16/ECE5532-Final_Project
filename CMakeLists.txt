cmake_minimum_required(VERSION 2.8.3)
project(final_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  ugv_course_libs
  dynamic_reconfigure
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(final_project src/final_project.cpp)
target_link_libraries(final_project  ${catkin_LIBRARIES})

