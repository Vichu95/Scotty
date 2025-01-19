#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import LoadController, SwitchController

class StandController:
    def __init__(self):
        rospy.init_node("stand_controller", anonymous=True)
        
        # Initialize service proxies for controller manager
        rospy.wait_for_service("/controller_manager/load_controller")
        self.load_controller_srv = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)

        rospy.wait_for_service("/controller_manager/switch_controller")
        self.switch_controller_srv = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

        # Publisher for joint group position controller
        self.joint_pub = rospy.Publisher("/joint_group_position_controller/command", JointTrajectory, queue_size=10)
        
        rospy.loginfo("StandController initialized.")

    def load_and_start_controller(self):
        # Load the controller
        try:
            rospy.loginfo("Loading controller...")
            load_response = self.load_controller_srv("joint_group_position_controller")
            if load_response.ok:
                rospy.loginfo("Controller loaded successfully.")
            else:
                rospy.logerr("Failed to load controller.")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return False

        # Start the controller
        try:
            rospy.loginfo("Starting controller...")
            switch_response = self.switch_controller_srv(
                start_controllers=["joint_group_position_controller"],
                stop_controllers=[],
                strictness=2
            )
            if switch_response.ok:
                rospy.loginfo("Controller started successfully.")
                return True
            else:
                rospy.logerr("Failed to start controller.")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return False

    def stand_up(self):
        # Define the joint names and desired positions for the standing pose
        joint_names = [
            "abad_FL_joint", "hip_FL_joint", "knee_FL_joint",
            "abad_FR_joint", "hip_FR_joint", "knee_FR_joint",
            "abad_RL_joint", "hip_RL_joint", "knee_RL_joint",
            "abad_RR_joint", "hip_RR_joint", "knee_RR_joint"
        ]

        joint_positions = [
            0.0, 0.6, -1.2,  # Front Left
            0.0, 0.6, -1.2,  # Front Right
            0.0, 0.6, -1.2,  # Rear Left
            0.0, 0.6, -1.2   # Rear Right
        ]

        # Create and publish the trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(3.0)  # Smooth transition over 3 seconds

        traj_msg.points = [point]

        # Publish the trajectory message
        rospy.sleep(1)
        rospy.loginfo("Publishing trajectory message for stand-up position...")
        self.joint_pub.publish(traj_msg)
        rospy.sleep(5)  # Allow time for the robot to complete the motion

if __name__ == "__main__":
    try:
        controller = StandController()
        if controller.load_and_start_controller():
            controller.stand_up()
    except rospy.ROSInterruptException:
        pass
