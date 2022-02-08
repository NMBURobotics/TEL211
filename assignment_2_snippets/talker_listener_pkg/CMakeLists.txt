cmake_minimum_required(VERSION 3.0.2)

project(talker_listener_pkg)

find_package(catkin REQUIRED COMPONENTS 
roscpp
rospy
std_msgs 
geometry_msgs)
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
    CATKIN_DEPENDS roscpp  std_msgs geometry_msgs)

catkin_install_python(PROGRAMS
                      src/talker.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                      )

catkin_install_python(PROGRAMS
                    src/listener.py
                    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                    )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
