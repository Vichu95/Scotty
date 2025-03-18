#!/usr/bin/env python

"""
Project     : Scotty
ROS Package : scotty_controller
Script Name : scotty_state_down.py
Author      : Vishnudev Kurumbaparambil
Organization: Hochschule Anhalt
Description : Handles the state Down
Usage       : These scripts are run from the scotty_controller.launch
Logic       : It publishes the joint values for Down pose after starting the joint_group_position_controller
"""

import rospy
import sys
import numpy as np
from scotty_controller_manager import ScottyControllerManager
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import LoadController, SwitchController
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class DownController:
    def __init__(self):
        rospy.init_node("scotty_state_controller_down", anonymous=True)
        
        # Publisher for joint group position controller
        self.controller_manager = ScottyControllerManager()
        self.joint_pub = rospy.Publisher("/joint_group_position_controller/command", JointTrajectory, queue_size=10)
        
        # Publisher for indicating state change is finished
        self.state_execution_status_pub = rospy.Publisher("/scotty_controller/state_execution_status", String, queue_size=10)
        
        # Publisher to broadcast logs to the GUI console
        self.console_log_pub = rospy.Publisher("/scotty_controller/console_log", String, queue_size=10)

        # Initialize current joint positions
        self.current_positions = None
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        rospy.loginfo("DownController : Initialized.")

    def joint_state_callback(self, msg):
        """ Callback to store the latest joint states. """
        self.current_positions = dict(zip(msg.name, msg.position))
        # rospy.loginfo("Received joint states: {}".format(self.current_positions))

    def smooth_down(self, step_size=0.05):
        """ Moves smoothly from the current joint positions to the down position. """

        # Load and start the controller
        if not self.controller_manager.load_and_start_controller("joint_group_position_controller"):
            rospy.logerr("DownController : Failed to load and start the controller.")
            return
        
        # Define the joint names and target positions for the down pose
        joint_names = [
            "abad_FL_joint", "hip_FL_joint", "knee_FL_joint",
            "abad_FR_joint", "hip_FR_joint", "knee_FR_joint",
            "abad_RL_joint", "hip_RL_joint", "knee_RL_joint",
            "abad_RR_joint", "hip_RR_joint", "knee_RR_joint"
        ]

        target_positions = [
            0.2, 1.04, -2.18,  # Front Left
           -0.2, 1.04, -2.18,  # Front Right
            0.2, 1.04, -2.18,  # Rear Left
           -0.2, 1.04, -2.18   # Rear Right
        ]

        rospy.loginfo("DownController : Waiting for current joint states...")

        # Wait until we receive valid joint states
        while self.current_positions is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("DownController : Received current joint states.")

        # Extract the current joint positions in the correct order
        current_positions = [self.current_positions[joint] for joint in joint_names]
        rospy.loginfo("Current joint positions: {}".format(current_positions))
        rospy.loginfo("Target joint positions: {}".format(target_positions))


        # Compute the number of steps needed for smooth transition
        max_diff = max(abs(np.array(target_positions) - np.array(current_positions)))
        steps = int(np.ceil(max_diff / step_size))

        rospy.loginfo("DownController : Moving smoothly in {} steps.".format(steps))


        for i in range(steps + 1):
            alpha = i / float(steps)  # Interpolation factor (0 to 1)
            intermediate_positions = (1 - alpha) * np.array(current_positions) + alpha * np.array(target_positions)

            traj_msg = JointTrajectory()
            traj_msg.joint_names = joint_names
            traj_msg.header.stamp = rospy.Time.now()

            point = JointTrajectoryPoint()
            point.positions = intermediate_positions.tolist()
            point.time_from_start = rospy.Duration(alpha * 2.0)  # Scale duration smoothly

            traj_msg.points = [point]
            
            rospy.loginfo("Step {}/{} | Interpolated positions: {}".format(i,steps,intermediate_positions.tolist()))

            # Publish each step separately
            self.console_log_pub.publish("INFO    : Publishing trajectory message")
            self.joint_pub.publish(traj_msg)
            rospy.sleep(0.1)  # Small delay to allow smooth execution

        rospy.loginfo("DownController : Successfully reached down position.")

        # Indicate completion
        self.state_execution_status_pub.publish("Down_Done")
        rospy.loginfo("DownController : Completed down position...")

    def down(self):
        rospy.loginfo("DownController : Mode is Simulation")
        # Load and start the controller
        if not self.controller_manager.load_and_start_controller("joint_group_position_controller"):
            rospy.logerr("DownController : Failed to load and start the controller.")
            return

        # Define the joint names and desired positions for the down pose
        joint_names = [
            "abad_FL_joint", "hip_FL_joint", "knee_FL_joint",
            "abad_FR_joint", "hip_FR_joint", "knee_FR_joint",
            "abad_RL_joint", "hip_RL_joint", "knee_RL_joint",
            "abad_RR_joint", "hip_RR_joint", "knee_RR_joint"
        ]

        joint_positions = [
            0.2, 1.04, -2.18,  # Front Left
           -0.2, 1.04, -2.18,  # Front Right
            0.2, 1.04, -2.18,  # Rear Left
           -0.2, 1.04, -2.18   # Rear Right
        ]

        # Create and publish the trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(2.0)  # Smooth transition over 2 seconds

        traj_msg.points = [point]

        # Publish the trajectory message
        rospy.sleep(1)
        rospy.loginfo("DownController : Publishing trajectory message")
        self.console_log_pub.publish("INFO    : Publishing trajectory message")
        self.joint_pub.publish(traj_msg)
        rospy.sleep(3)  # Allow time for the robot to complete the motion

        self.state_execution_status_pub.publish("Down_Done")
        rospy.loginfo("DownController : Completed down position...")

if __name__ == "__main__":
    try:
        mode = sys.argv[1] if len(sys.argv) > 1 else "simulation" # Default mode simulation
        controller = DownController()

        if mode =="hardware":
            controller.smooth_down()
        else:
            controller.down()

    except rospy.ROSInterruptException:
        pass
