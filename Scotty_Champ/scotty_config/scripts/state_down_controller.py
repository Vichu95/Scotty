#!/usr/bin/env python

import rospy
from scotty_controller_manager import ScottyControllerManager
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import LoadController, SwitchController
from std_msgs.msg import String

class DownController:
    def __init__(self):
        rospy.init_node("down_controller", anonymous=True)
        
        # Publisher for joint group position controller
        self.controller_manager = ScottyControllerManager()
        self.joint_pub = rospy.Publisher("/joint_group_position_controller/command", JointTrajectory, queue_size=10)
        
        # Publisher for indicating state change is finished
        self.state_execution_status_pub = rospy.Publisher("/scotty_controller/state_execution_status", String, queue_size=10)
        
        rospy.loginfo("DownController initialized.")

    def down(self):

        # Load and start the controller
        if not self.controller_manager.load_and_start_controller("joint_group_position_controller"):
            rospy.logerr("Failed to load and start the controller.")
            return

        # Define the joint names and desired positions for the down pose
        joint_names = [
            "abad_FL_joint", "hip_FL_joint", "knee_FL_joint",
            "abad_FR_joint", "hip_FR_joint", "knee_FR_joint",
            "abad_RL_joint", "hip_RL_joint", "knee_RL_joint",
            "abad_RR_joint", "hip_RR_joint", "knee_RR_joint"
        ]

        ##Joint values from champ, when the robot down
        # joint_positions = [
        #     -0.24987684190273285, 0.937517523765564, -1.8144596815109253, 
        #     0.23306728899478912, 0.9333722591400146, -1.806188941001892, 
        #     -0.24987684190273285, 0.9375174045562744, -1.8144596815109253, 
        #     0.23306728899478912, 0.9333721399307251, -1.806188941001892]


        joint_positions = [
            0.2, 1.04, -2.18,  # Front Left
           -0.2, 1.04, -2.18, # Front Right
            0.2, 1.04, -2.18,  # Rear Left
           -0.2, 1.04, -2.18   # Rear Right
        ]

        # Create and publish the trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(2.0)  # Smooth transition over 3 seconds

        traj_msg.points = [point]

        # Publish the trajectory message
        rospy.sleep(1)
        rospy.loginfo("Publishing trajectory message for down position...")
        self.joint_pub.publish(traj_msg)
        rospy.sleep(3)  # Allow time for the robot to complete the motion

        self.state_execution_status_pub.publish("Down_Done")
        rospy.loginfo("Completed down position...")

if __name__ == "__main__":
    try:
        controller = DownController()
        controller.down()

    except rospy.ROSInterruptException:
        pass
