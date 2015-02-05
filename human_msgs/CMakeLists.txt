cmake_minimum_required(VERSION 2.8.3)
project(human_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  rospy
  std_msgs
)

 add_action_files(
   FILES
   human_following.action
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
   geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
