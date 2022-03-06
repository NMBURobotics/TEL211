cmake_minimum_required(VERSION 3.0.2)
project(simple_nav_pkg)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  message_runtime
  rospy
  std_msgs
)

# catkin_python_setup()

# Generate actions in the 'action' folder
 add_action_files(
   FILES
   Nav.action
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   geometry_msgs
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_nav_pkg
#  CATKIN_DEPENDS actionlib geometry_msgs message_generation message_runtime rospy std_msgs
#  DEPENDS system_lib
)
 catkin_install_python(PROGRAMS
   src/simple_action_server.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
