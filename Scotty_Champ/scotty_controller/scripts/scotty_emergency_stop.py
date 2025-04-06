#!/usr/bin/env python

"""
Project     : Scotty
ROS Package : scotty_controller
Script Name : scotty_emergency_stop.py
Author      : Vishnudev Kurumbaparambil
Organization: Hochschule Anhalt
Description : This script handles the emergency shutdown of Scotty
Usage       : These scripts are run from the scotty_controller.launch
"""

import rospy
import rosnode
import os
from std_msgs.msg import Bool
import subprocess


class EmergencyStop:
    def __init__(self):
        rospy.init_node('emergency_stop', anonymous=False)
        rospy.loginfo("Emergency Stop Node is Initialized")

        # Subscriber to listen for emergency stop signal
        self.emergency_sub = rospy.Subscriber('/scotty_controller/emergency_stop', Bool, self.handle_emergency)

    def handle_emergency(self, msg):
        """Callback for emergency stop signal."""
        if msg.data:
            rospy.logwarn("Emergency Stop Triggered! Killing all ROS nodes...")
            self.kill_all_ros_nodes()
        else:
            rospy.loginfo("Emergency Stop signal received, but not activated.")

    def kill_all_ros_nodes(self):
        """Kills all active ROS nodes."""
        try:

            os.system("pkill -9 -f '/rosout'")
            os.system("pkill -9 -f 'python.*server_launch.py'")
            os.system("pkill -9 -f 'rosbridge_websocket'")

            # Kills the main controller
            os.system("pkill -9 -f '/scotty_main_controller'")

            # Kills the gazebo process
            subprocess.call(["pkill", "-f", "gzserver"])
            subprocess.call(["pkill", "-f", "gzclient"])
            # List all active ROS nodes
            nodes = rosnode.get_node_names()
            for node in nodes:
                rospy.loginfo("Killing node: {}".format(node))
                os.system("rosnode kill {}".format(node))
            rospy.loginfo("All ROS nodes terminated successfully.")
        except Exception as e:
            rospy.logerr("Failed to kill nodes: {}".format(e))

if __name__ == "__main__":
    try:
        EmergencyStop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass