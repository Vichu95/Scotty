cmake_minimum_required(VERSION 2.8.3)
project(scotty_config)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs)


catkin_package()

install(PROGRAMS
    scripts/main_controller.py
    scripts/scotty_controller_manager.py
    scripts/state_idle_startup.py
    scripts/stand_controller.py
    scripts/state_walk.py    
    scripts/state_down_controller.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)