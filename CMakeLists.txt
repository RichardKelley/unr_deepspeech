cmake_minimum_required(VERSION 2.8.3)
project(unr_deepspeech)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
   FILES Listen.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
