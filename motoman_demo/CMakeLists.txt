cmake_minimum_required(VERSION 2.8.3)
project(motoman_demo)
find_package(catkin REQUIRED
  rospy
  moveit_commander
  tf
  motoman_demo_msgs
  sensor_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  )

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
