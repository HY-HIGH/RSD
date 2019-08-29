cmake_minimum_required(VERSION 2.8.3)
project(people_detection)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  rospy
  roscpp
)


add_message_files(
  FILES 
  MsgState.msg
  )



 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )


catkin_package(

  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs

)

include_directories(
 #include
  ${catkin_INCLUDE_DIRS}
)

