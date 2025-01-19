#!/usr/bin/env python

import rospy
import subprocess
from std_msgs.msg import String


class MainController:
    def __init__(self):
        rospy.init_node("scotty_main_controller")
        rospy.loginfo("Starting Scotty Main Controller...")

        # Define the states
        self.states = ["idle", "stand_up"]
        self.current_state_index = 0

        # Publisher to broadcast current state
        self.state_pub = rospy.Publisher("/robot_state", String, queue_size=10)

        # Start the initial state
        self.start_current_state()

    def start_current_state(self):
        """Start the current state based on the state index."""
        current_state = self.states[self.current_state_index]
        rospy.loginfo("Entering state: {}".format(current_state))

        # Publish the current state
        self.state_pub.publish(current_state)

        if current_state == "idle":
            self.start_idle_state()
        elif current_state == "stand_up":
            self.start_stand_up_state()
        else:
            rospy.logwarn("Unknown state: {}".format(current_state))

    def start_idle_state(self):
        """Start the idle state."""
        rospy.loginfo("Launching idle state node...")
        try:
            idle_process = subprocess.Popen(["rosrun", "scotty_config", "state_idle_startup.py"])
            idle_process.wait()  # Wait for the idle state process to complete
            rospy.loginfo("Idle state completed.")

            # Transition to the next state
            self.transition_to_next_state()
        except Exception as e:
            rospy.logerr("Failed to launch idle state node: {}".format(e))

    def start_stand_up_state(self):
        """Start the stand-up state (currently a placeholder)."""
        rospy.loginfo("Executing stand-up state...")
        rospy.sleep(3)  # Simulate some processing delay
        rospy.loginfo("Stand-up state complete.")

        # Transition to the next state (if applicable)
        self.transition_to_next_state()

    def transition_to_next_state(self):
        """Move to the next state if available."""
        if self.current_state_index < len(self.states) - 1:
            self.current_state_index += 1
            self.start_current_state()
        else:
            rospy.loginfo("All states have been completed. Stopping controller.")
            rospy.signal_shutdown("Controller execution complete.")


if __name__ == "__main__":
    try:
        controller = MainController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

