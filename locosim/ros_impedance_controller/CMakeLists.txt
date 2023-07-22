cmake_minimum_required(VERSION 2.8.3)
project(ros_impedance_controller)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")
# ROS

find_package( Eigen3 REQUIRED )

find_package(catkin REQUIRED tf controller_interface
            realtime_tools sensor_msgs message_generation gazebo_msgs  geometry_msgs     REQUIRED)


add_message_files(
  DIRECTORY
  msg
  FILES
  pid.msg
  BaseState.msg
  ContactsState.msg
  Forces.msg
  EffortPid.msg
)

add_service_files(
  DIRECTORY
  srv
  FILES
  set_pids.srv
  get_map.srv
  get_effort_pid.srv
  generic_float.srv
  MoveJoints.srv
  MoveTo.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${GAZEBO_LIBRARIES}
  CATKIN_DEPENDS message_runtime sensor_msgs geometry_msgs


)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include  ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${ros_impedance_controller_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES controller_plugin.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  PATTERN ".svn" EXCLUDE)

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
