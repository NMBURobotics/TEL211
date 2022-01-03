cmake_minimum_required(VERSION 3.0.2)

project(talker_listener_pkg)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

catkin_install_python(PROGRAMS
                      talker_listener_pkg/talker.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                      )