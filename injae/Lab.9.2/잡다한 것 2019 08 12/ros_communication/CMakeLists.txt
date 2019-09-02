cmake_minimum_required(VERSION 2.8.3)
project(ros_communication)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)


add_message_files(FILES Position.msg) 
add_message_files(FILES State.msg) 
add_service_files(FILES SetPosition.srv) 
add_service_files(FILES SetState.srv) 

generate_messages(DEPENDENCIES std_msgs)
catkin_package(
LIBRARIES ros_communication
CATKIN_DEPENDS std_msgs roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(node1 src/node1.cpp) 
add_dependencies(node1 ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
target_link_libraries(node1 ${catkin_LIBRARIES})


add_executable(node2 src/node2.cpp)
add_dependencies(node2 ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
target_link_libraries(node2 ${catkin_LIBRARIES})

add_executable(node3 src/node3.cpp)
add_dependencies(node3 ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
target_link_libraries(node3 ${catkin_LIBRARIES})
