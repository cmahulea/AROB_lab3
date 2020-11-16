cmake_minimum_required(VERSION 3.0.2)
project(AROB_lab3)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  nav_msgs
  roscpp
  tf
  message_generation
  actionlib_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  my_msg.msg
)



## Generate services in the 'srv' folder
 add_service_files(DIRECTORY srv
   FILES
   arob_lab3.srv
)

generate_messages(DEPENDENCIES std_msgs )
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES AROB_lab3
  CATKIN_DEPENDS geometry_msgs std_msgs nav_msgs roscpp tf message_runtime
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(teleop_twist_keyboard src/teleop_twist_keyboard.cpp)
add_executable(lowcontrol src/lowcontrol.cpp)
add_executable(robot_location src/robot_location.cpp)
add_executable(followTargets src/followTargets.cpp)
add_executable(highcontrol src/highcontrol.cpp)
add_executable(mymap_server src/mymap_server.cpp)

target_link_libraries(teleop_twist_keyboard ${catkin_LIBRARIES})
target_link_libraries(lowcontrol ${catkin_LIBRARIES})
target_link_libraries(robot_location ${catkin_LIBRARIES})
target_link_libraries(followTargets ${catkin_LIBRARIES})
target_link_libraries(highcontrol ${catkin_LIBRARIES})
target_link_libraries(mymap_server ${catkin_LIBRARIES})

#add_dependencies(mymap_server AROB_lab3_generate_messages_cpp)
add_dependencies(mymap_server ${AROB_lab3_EXPORTED_TARGETS})
add_dependencies(mymap_server ${catkin_EXPORTED_TARGETS})


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
