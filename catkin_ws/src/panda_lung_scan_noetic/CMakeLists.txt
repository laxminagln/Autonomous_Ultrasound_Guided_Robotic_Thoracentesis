cmake_minimum_required(VERSION 3.0.2)
project(panda_lung_scan_noetic)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
  geometry_msgs
  controller_interface
  franka_hw
  hardware_interface
  pluginlib
  controller_manager_msgs
  visualization_msgs
  nav_msgs
  tf
  tf2_ros
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES panda_ultrasound_force_scan_controller
  CATKIN_DEPENDS
    roscpp
    moveit_ros_planning_interface
    geometry_msgs
    controller_interface
    franka_hw
    hardware_interface
    pluginlib
  controller_manager_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(scan_preview src/scan_preview.cpp)
target_link_libraries(scan_preview ${catkin_LIBRARIES})
add_dependencies(scan_preview ${catkin_EXPORTED_TARGETS})

add_executable(scan_moveit_preview src/scan_moveit_preview.cpp)
target_link_libraries(scan_moveit_preview ${catkin_LIBRARIES})
add_dependencies(scan_moveit_preview ${catkin_EXPORTED_TARGETS})

add_executable(scan_moveit_real src/scan_moveit_real.cpp)
target_link_libraries(scan_moveit_real ${catkin_LIBRARIES})
add_dependencies(scan_moveit_real ${catkin_EXPORTED_TARGETS})

add_library(panda_ultrasound_force_scan_controller
  src/panda_ultrasound_force_scan_controller.cpp
)
target_link_libraries(panda_ultrasound_force_scan_controller ${catkin_LIBRARIES})
add_dependencies(panda_ultrasound_force_scan_controller ${catkin_EXPORTED_TARGETS})

add_executable(move_to_scan_start src/move_to_scan_start.cpp)
target_link_libraries(move_to_scan_start ${catkin_LIBRARIES})
add_dependencies(move_to_scan_start ${catkin_EXPORTED_TARGETS})

add_executable(get_current_pose src/get_current_pose.cpp)
target_link_libraries(get_current_pose ${catkin_LIBRARIES})
add_dependencies(get_current_pose ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS
  scan_preview
  scan_moveit_preview
  scan_moveit_real
  move_to_scan_start
  panda_ultrasound_force_scan_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  panda_ultrasound_force_scan_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)