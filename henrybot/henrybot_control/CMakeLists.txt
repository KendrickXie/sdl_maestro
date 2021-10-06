cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(henrybot_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################


# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
)


catkin_install_python(PROGRAMS
  scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})