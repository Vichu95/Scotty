#!/usr/bin/env python

"""
Project     : Scotty
ROS Package : scotty_controller
Script Name : scotty_state_stand.py
Author      : Vishnudev Kurumbaparambil
Organization: Hochschule Anhalt
Description : Handles the state Stand
Usage       : These scripts are run from the scotty_controller.launch
Logic       : It publishes the joint values for Stand pose after starting the joint_group_position_controller
"""

import rospy
from scotty_controller_manager import ScottyControllerManager
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

class StandController:
    def __init__(self):
        rospy.init_node("scotty_state_controller_stand")
        
        # Publisher for joint group position controller
        self.controller_manager = ScottyControllerManager()
        self.joint_pub = rospy.Publisher("/joint_group_position_controller/command", JointTrajectory, queue_size=10)
        
        # Publisher for indicating state change is finished
        self.state_change_pub = rospy.Publisher("/scotty_controller/state_execution_status", String, queue_size=10)
        
        # Publisher to broadcast logs to the GUI console
        self.console_log_pub = rospy.Publisher("/scotty_controller/console_log", String, queue_size=10)

        rospy.loginfo("StandController : Initialized")

    def stand_up(self):

        # Load and start the controller
        if not self.controller_manager.load_and_start_controller("joint_group_position_controller"):
            rospy.logerr("StandController : Failed to load and start the controller.")
            return

        # Define the joint names and desired positions for the standing pose
        joint_names = [
            "abad_FL_joint", "hip_FL_joint", "knee_FL_joint",
            "abad_FR_joint", "hip_FR_joint", "knee_FR_joint",
            "abad_RL_joint", "hip_RL_joint", "knee_RL_joint",
            "abad_RR_joint", "hip_RR_joint", "knee_RR_joint"
        ]

        ##Joint values from champ, when the robot stands
        joint_positions = [
            -0.24987684190273285, 0.937517523765564, -1.8144596815109253, 
             0.23306728899478912, 0.9333722591400146, -1.806188941001892, 
            -0.24987684190273285, 0.9375174045562744, -1.8144596815109253, 
             0.23306728899478912, 0.9333721399307251, -1.806188941001892]


        # Create and publish the trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(2.0)  # Smooth transition over 2 seconds

        traj_msg.points = [point]

        # Publish the trajectory message
        rospy.sleep(1)
        rospy.loginfo("StandController : Publishing trajectory message")
        self.console_log_pub.publish("INFO    : Publishing trajectory message")
        self.joint_pub.publish(traj_msg)
        rospy.sleep(3)  # Allow time for the robot to complete the motion

        self.state_change_pub.publish("Stand_Done")

if __name__ == "__main__":
    try:
        controller = StandController()        
        controller.stand_up()

    except rospy.ROSInterruptException:
        pass
