cmake_minimum_required(VERSION 2.8.3)
project(scotty_config)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs)


catkin_package()

install(PROGRAMS
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
