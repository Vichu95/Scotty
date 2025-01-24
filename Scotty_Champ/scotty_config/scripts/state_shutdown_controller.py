#!/usr/bin/env python

import rospy
from scotty_controller_manager import ScottyControllerManager
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import LoadController, SwitchController
from std_msgs.msg import String

class ShutdownController:
    def __init__(self):
        rospy.init_node("shutdown_controller", anonymous=True)
        
        # Publisher for joint group position controller
        self.controller_manager = ScottyControllerManager()
        self.joint_pub = rospy.Publisher("/joint_group_position_controller/command", JointTrajectory, queue_size=10)
        
        # Publisher for indicating state change is finished
        self.state_execution_status_pub = rospy.Publisher("/scotty_controller/state_execution_status", String, queue_size=10)
        
        rospy.loginfo("ShutdownController initialized.")

    def shutdown(self):

        # Load and start the controller
        if not self.controller_manager.load_and_start_controller("joint_group_position_controller"):
            rospy.logerr("Failed to load and start the controller.")
            return

        # Define the joint names and desired positions for the shutdown pose
        joint_names = [
            "abad_FL_joint", "hip_FL_joint", "knee_FL_joint",
            "abad_FR_joint", "hip_FR_joint", "knee_FR_joint",
            "abad_RL_joint", "hip_RL_joint", "knee_RL_joint",
            "abad_RR_joint", "hip_RR_joint", "knee_RR_joint"
        ]

        joint_positions = [
             0.65, 1.5, -2.48,  # Front Left
            -0.65, 1.5, -2.8, # Front Right
             0.65, 1.8, -2.5,  # Rear Left
            -0.65, 1.8, -2.5   # Rear Right
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
        rospy.loginfo("Publishing trajectory message for shutdown position...")
        self.joint_pub.publish(traj_msg)
        rospy.sleep(3)  # Allow time for the robot to complete the motion


        # Stop the controller
        if not self.controller_manager.stop_controller("joint_group_position_controller"):
            rospy.logerr("Failed to stop the controller.")

        self.shutdown_gazebo()

        self.state_execution_status_pub.publish("Shutdown_Done")
        rospy.loginfo("Completed shutdown position...")

    def shutdown_gazebo(self):
        """Shuts down Gazebo by killing gzserver and gzclient processes."""
        try:
            # Check and kill gzserver
            gzserver_running = subprocess.call(["pgrep", "-f", "gzserver"])
            if gzserver_running == 0:
                subprocess.call(["pkill", "-f", "gzserver"])
                rospy.loginfo("Successfully terminated gzserver.")
            else:
                rospy.logwarn("gzserver not running.")

            # Check and kill gzclient
            gzclient_running = subprocess.call(["pgrep", "-f", "gzclient"])
            if gzclient_running == 0:
                subprocess.call(["pkill", "-f", "gzclient"])
                rospy.loginfo("Successfully terminated gzclient.")
            else:
                rospy.logwarn("gzclient not running.")

            # Give processes time to terminate
            time.sleep(2)

            # Verify processes are terminated
            gzserver_running = subprocess.call(["pgrep", "-f", "gzserver"])
            gzclient_running = subprocess.call(["pgrep", "-f", "gzclient"])
            if gzserver_running != 0 and gzclient_running != 0:
                rospy.loginfo("Gazebo completely shut down.")
            else:
                rospy.logerr("Failed to completely terminate Gazebo processes.")
        except Exception as e:
            rospy.logerr("Error shutting down Gazebo: {}".format(e))


if __name__ == "__main__":
    try:
        controller = ShutdownController()
        controller.shutdown()

    except rospy.ROSInterruptException:
        pass
