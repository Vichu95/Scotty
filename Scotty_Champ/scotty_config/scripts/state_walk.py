#!/usr/bin/env python

import rospy
import subprocess

class WalkController:
    def __init__(self):
        rospy.init_node('walk_controller', anonymous=True)
        rospy.loginfo("Walk Controller initialized.")

    def start_champ_controller(self):
        rospy.loginfo("Starting CHAMP controller via launch file.")
        # Replace 'path_to_champ_controller.launch' with the correct path
        subprocess.Popen(["roslaunch", "scotty_config", "bringup.launch"])
        rospy.loginfo("CHAMP controller started. Ready for teleop.")

if __name__ == '__main__':
    try:
        controller = WalkController()
        controller.start_champ_controller()
    except rospy.ROSInterruptException:
        pass
