#!/usr/bin/env python

"""
Project     : Scotty
ROS Package : scotty_controller
Script Name : scotty_state_walk.py
Author      : Vishnudev Kurumbaparambil
Organization: Hochschule Anhalt
Description : Handles the state Walk
Usage       : These scripts are run from the scotty_controller.launch
Logic       : Calls the champ bring up and champ takes over the walking
"""

import rospy
import subprocess
import sys
from std_msgs.msg import String

class WalkController:
    def __init__(self):
        rospy.init_node('scotty_state_controller_walk', anonymous=True)
        rospy.loginfo("WalkController : Initialized.")

        # Publisher to broadcast logs to the GUI console
        self.console_log_pub = rospy.Publisher("/scotty_controller/console_log", String, queue_size=10)

    def start_champ_controller(self,mode):
        rospy.loginfo("WalkController : Starting CHAMP controller via bringup.launch")

        if mode =="hardware":
            subprocess.Popen(["roslaunch", "scotty_config", "bringup.launch", "hardware_connected:=true"])
        else:
            subprocess.Popen(["roslaunch", "scotty_config", "bringup.launch"])

        rospy.loginfo("WalkController : CHAMP controller started. Ready for teleop.")
        self.console_log_pub.publish("INFO    : Use teleop for moving the robot")

if __name__ == '__main__':
    try:
        mode = sys.argv[1] if len(sys.argv) > 1 else "simulation" # Default mode simulation
        controller = WalkController()
        controller.start_champ_controller(mode)
    except rospy.ROSInterruptException:
        pass
