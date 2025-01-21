#!/usr/bin/env python

import rospy
import subprocess
import time
from std_msgs.msg import String
import webbrowser
import rospkg


class MainController:
    def __init__(self):
        rospy.init_node("scotty_main_controller")
        rospy.loginfo("Starting Scotty Main Controller...")

        ## Open the GUI
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("scotty_config")
        webbrowser.open(package_path + "/scripts/index.html")

        # Define the states
        self.states = ["Idle", "Ready", "Stand", "Walk"]
        self.current_state_index = 0

        # Publisher to broadcast current state
        self.state_pub = rospy.Publisher("/scotty_controller/state", String, queue_size=10)

        # Subscribe to state change command
        self.state_change_sub = rospy.Subscriber("/scotty_controller/change_state", String, self.change_state_callback)

        # Start the initial state
        self.start_current_state()

    def start_current_state(self):
        """Start the current state based on the state index."""
        current_state = self.states[self.current_state_index]
        rospy.loginfo("Entering state: {}".format(current_state))

        # Publish the current state
        self.state_pub.publish(current_state)

        if current_state == "Idle":
            self.start_idle_state()
        elif current_state == "Ready":
            self.start_ready_state()
        elif current_state == "Stand":
            self.start_stand_up_state()
        elif current_state == "Walk":
            self.start_walk_state()
        else:
            rospy.logwarn("Unknown state: {}".format(current_state))

    def start_idle_state(self):
        """Start the Idle state."""
        rospy.loginfo("Launching Idle state node...")
        try:
            idle_process = subprocess.Popen(["rosrun", "scotty_config", "state_idle_startup.py"])
            idle_process.wait()  # Wait for the Idle state process to complete
            rospy.loginfo("Idle state completed.")

            ## Going to ready state automatically in 5s
            rospy.sleep(5)
            self.current_state_index += 1
            self.start_current_state()

        except Exception as e:
            rospy.logerr("Failed to launch Idle state node: {}".format(e))

    def start_ready_state(self):
        """Start Ready state."""
        rospy.loginfo("In Ready state node...")
        rospy.sleep(1)
        # Stay in Ready state
        self.start_current_state()

    def start_stand_up_state(self):
        """Start the Stand-up state (currently a placeholder)."""
        rospy.loginfo("Launching Stand state node")
        try:
            stand_process = subprocess.Popen(["rosrun", "scotty_config", "stand_controller.py"])
            stand_process.wait()  # Wait for the Stand state process to complete
            rospy.loginfo("Stand state completed.")

        except Exception as e:
            rospy.logerr("Failed to launch Stand state node: {}".format(e))

    def start_walk_state(self):
        """Start the Walk state (currently a placeholder)."""
        rospy.loginfo("Launching Walk state node")
        try:
            stand_process = subprocess.Popen(["rosrun", "scotty_config", "state_walk.py"])
            stand_process.wait()  # Wait for the Walk state process to complete
            rospy.loginfo("Walk state completed.")

        except Exception as e:
            rospy.logerr("Failed to launch Walk state node: {}".format(e))

    def change_state_callback(self, msg):
        """Move to the next state if available."""
        desired_state = msg.data
        current_state = self.states[self.current_state_index ]

        rospy.loginfo("\n\nRECEIVED")

        valid_transitions = {
            "Ready"  : "Stand",
            "Stand"  : "Walk" ,

        }

        if current_state in valid_transitions and valid_transitions[current_state] == desired_state:
            rospy.loginfo("Transitioning from {} to {}.".format(current_state,desired_state))
            self.current_state_index += 1
            self.start_current_state()
        else:
            rospy.loginfo("All states have been completed. Stopping controller.")
            # rospy.signal_shutdown("Controller execution complete.")


if __name__ == "__main__":
    try:
        controller = MainController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

