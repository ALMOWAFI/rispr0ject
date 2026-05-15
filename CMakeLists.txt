cmake_minimum_required(VERSION 3.0.2)
project(memory_game)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  image_transport
  image_geometry
  tf2_geometry_msgs
  message_generation
  tf
  tf2_ros
)

## System dependencies (optional - only needed when implementing vision)
find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)
# find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

## Declare messages
add_message_files(
  FILES
  Block.msg
  BlockArray.msg
  BlockSequence.msg
  PlayerSelection.msg
)

## Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

## Declare this project as a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs visualization_msgs cv_bridge image_transport image_geometry tf2_geometry_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Add executables
add_executable(vision_node src/vision_node.cpp)
target_link_libraries(vision_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(game_node src/game_node.cpp)
target_link_libraries(game_node ${catkin_LIBRARIES})
add_dependencies(game_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(motion_node src/motion_node.cpp)
target_link_libraries(motion_node ${catkin_LIBRARIES})
add_dependencies(motion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 add_executable(motion_hw_node newmotion/motion_hw_node.cpp)
target_link_libraries(motion_hw_node ${catkin_LIBRARIES})
add_dependencies(motion_hw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Optional: build MoveIt-backed motion bridge if MoveIt is installed.
find_package(moveit_ros_planning_interface QUIET)
find_package(actionlib QUIET)
find_package(franka_gripper QUIET)
if(moveit_ros_planning_interface_FOUND)
  add_executable(motion_moveit_node newmotion/motion_moveit_node.cpp)
  target_link_libraries(motion_moveit_node ${catkin_LIBRARIES} ${moveit_ros_planning_interface_LIBRARIES})
  target_include_directories(motion_moveit_node PRIVATE ${moveit_ros_planning_interface_INCLUDE_DIRS})
  if(actionlib_FOUND AND franka_gripper_FOUND)
    target_compile_definitions(motion_moveit_node PRIVATE MEMORY_GAME_HAS_FRANKA_GRIPPER=1)
    target_link_libraries(motion_moveit_node ${actionlib_LIBRARIES} ${franka_gripper_LIBRARIES})
    target_include_directories(motion_moveit_node PRIVATE ${actionlib_INCLUDE_DIRS} ${franka_gripper_INCLUDE_DIRS})
  else()
    target_compile_definitions(motion_moveit_node PRIVATE MEMORY_GAME_HAS_FRANKA_GRIPPER=0)
    message(STATUS "Franka gripper action not found: gripper close disabled in motion_moveit_node")
  endif()
  add_dependencies(motion_moveit_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  install(TARGETS motion_moveit_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  message(STATUS "MoveIt not found: skipping motion_moveit_node build")
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS vision_node game_node motion_node motion_hw_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (launch files, config files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

catkin_install_python(PROGRAMS
  scripts/test_player_selection.py
  scripts/player_selection.py
  scripts/vision_node_full_python.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_memory_game.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
