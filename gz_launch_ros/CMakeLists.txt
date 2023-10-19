cmake_minimum_required(VERSION 3.0.2)
project(gz_launch_ros)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  gazebo_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/worlds
  ${CMAKE_CURRENT_SOURCE_DIR}/models
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

roslaunch_add_file_check(launch)


