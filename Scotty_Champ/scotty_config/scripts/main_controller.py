#!/usr/bin/env python

import rospy
import subprocess

class MainController:
    def __init__(self):
        rospy.init_node("scotty_main_controller")
        rospy.loginfo("Starting Scotty Main Controller...")

        # Launch the idle state node
        self.start_idle_state()

    def start_idle_state(self):
        rospy.loginfo("Launching idle state node...")
        try:
            subprocess.Popen(["rosrun", "scotty_config", "state_idle_startup.py"])
            rospy.loginfo("Idle state node launched successfully.")
        except Exception as e:
            rospy.logerr("Failed to launch idle state node: {}".format(e))


if __name__ == "__main__":
    try:
        controller = MainController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass