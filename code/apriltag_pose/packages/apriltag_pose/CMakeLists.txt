cmake_minimum_required(VERSION 2.8.3)
project(apriltag_pose)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  geometry_msgs
)

catkin_package()
