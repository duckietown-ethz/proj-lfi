cmake_minimum_required(VERSION 2.8.3)
project(estimator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  #apriltags2_ros
  #std_msgs
  #geometry_msgs
  #message_generation
)

#add_dependencies(apriltag_pose ${catkin_EXPORTED_TARGETS})

#add_message_files(
#FILES
#AprilTagDetection.msg
#AprilTagDetectionArray.msg
#)

#generate_messages(
#DEPENDENCIES
#std_msgs
#geometry_msgs
#)

#catkin_package(
#CATKIN_DEPENDS message_runtime
#)

catkin_package()
