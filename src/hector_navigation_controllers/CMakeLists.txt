cmake_minimum_required(VERSION 2.8.3)
project(hector_navigation_controllers)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  hector_quadrotor_interface
  hector_quadrotor_actions
  hector_uav_msgs
  hector_navigation_msgs
  roscpp
  tf
  tf2_geometry_msgs
  visualization_msgs
  std_msgs
  geometry_msgs
  actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES hector_navigation_controllers
    CATKIN_DEPENDS roscpp
    DEPENDS
)

add_library(hector_navigation_controllers
  src/waypoint_controller.cpp
)

add_executable(waypoint_action
  src/waypoint_action.cpp
)
target_link_libraries(waypoint_action ${catkin_LIBRARIES})
add_dependencies(waypoint_action ${catkin_EXPORTED_TARGETS})

add_executable(followtrajectory_action
  src/follow_trajectory_action.cpp
)

target_link_libraries(followtrajectory_action ${catkin_LIBRARIES})
add_dependencies(followtrajectory_action ${catkin_EXPORTED_TARGETS})

target_link_libraries(hector_navigation_controllers ${catkin_LIBRARIES})
add_dependencies(hector_navigation_controllers ${catkin_EXPORTED_TARGETS})

install(TARGETS hector_navigation_controllers waypoint_action followtrajectory_action
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
