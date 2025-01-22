#!/usr/bin/env python

import rospy
import subprocess
import time
from std_msgs.msg import String
import webbrowser
import rospkg

## STATE MACHINE
scotty_states = 
{
    "Idle"  : "Idle",
    "Ready" : "Ready",
    "Down"  : "Down",
    "Stand" : "Stand",
    "Walk"  : "Walk",
    "Busy"  : "Busy"
}

valid_transitions = 
{
    scotty_states["Idle"]   : [scotty_states["Ready"]],
    scotty_states["Ready"]  : [scotty_states["Down"], scotty_states["Stand"]],
    scotty_states["Down"]   : [scotty_states["Stand"]],
    scotty_states["Stand"]  : [scotty_states["Down"], scotty_states["Walk"]],
    scotty_states["Walk"]   : [scotty_states["Stand"]],
    scotty_states["Busy"]   : [scotty_states["Idle"], scotty_states["Ready"], scotty_states["Down"], scotty_states["Stand"], scotty_states["Walk"]]

}


class MainController:
    def __init__(self):
        rospy.init_node("scotty_main_controller")
        rospy.loginfo("Starting Scotty Main Controller...")

        ## Open the GUI
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("scotty_config")
        webbrowser.open(package_path + "/scripts/index.html")

        self.current_state_key = 'Idle'

        # Publisher to broadcast current state
        self.state_pub = rospy.Publisher("/scotty_controller/state", String, queue_size=10)

        # Subscribe to state change command
        self.state_change_sub = rospy.Subscriber("/scotty_controller/change_state", String, self.change_state_callback)

        # Start the initial state
        self.start_current_state()

    def start_current_state(self):
        """Start the current state based on the state index."""
        current_state = scotty_states[self.current_state_key]
        rospy.loginfo("Entering state: {}".format(current_state))

        # Publish the current state
        self.state_pub.publish(current_state)

        if current_state == "Idle":
            self.start_idle_state()
        elif current_state == "Ready":
            self.start_ready_state()
        elif current_state == "Stand":
            self.start_stand_state()
        elif current_state == "Walk":
            self.start_walk_state()
        elif current_state == "Busy":
            # Do nothing
        else:
            rospy.logwarn("Unknown state requested: {}".format(current_state))

    ###########
    #  I D L E 
    ###########
    def start_idle_state(self):
        rospy.loginfo("Launching Idle state node...")
        try:
            idle_process = subprocess.Popen(["rosrun", "scotty_config", "state_idle_startup.py"])
            idle_process.wait()  # Wait for the Idle state process to complete
            rospy.loginfo("Idle state completed.")

            ## Going to ready state automatically in 3s
            rospy.sleep(3)
            self.current_state_key = 'Ready'
            self.start_current_state()

        except Exception as e:
            rospy.logerr("Failed to launch Idle state node: {}".format(e))

    ##############
    #   R E A D Y 
    ##############
    def start_ready_state(self):
        rospy.loginfo("In Ready state node...")
        rospy.sleep(1)
        # Stay in Ready state
        self.start_current_state()


    ##############
    #   D O W N 
    ##############
    def start_down_state(self):
        rospy.loginfo("Launching Down state node")
        try:
            stand_process = subprocess.Popen(["rosrun", "scotty_config", "scotty_state_down_controller.py"])
            stand_process.wait()  # Wait for the Down state process to complete
            rospy.loginfo("Down state completed.")

        except Exception as e:
            rospy.logerr("Failed to launch Down state node: {}".format(e))


    ##############
    #   S T A N D 
    ##############
    def start_stand_state(self):
        rospy.loginfo("Launching Stand state node")
        try:
            stand_process = subprocess.Popen(["rosrun", "scotty_config", "stand_controller.py"])
            stand_process.wait()  # Wait for the Stand state process to complete
            rospy.loginfo("Stand state completed.")

        except Exception as e:
            rospy.logerr("Failed to launch Stand state node: {}".format(e))


    ##############
    #   W A L K 
    ##############
    def start_walk_state(self):
        rospy.loginfo("Launching Walk state node")
        try:
            stand_process = subprocess.Popen(["rosrun", "scotty_config", "state_walk.py"])
            stand_process.wait()  # Wait for the Walk state process to complete
            rospy.loginfo("Walk state completed.")

        except Exception as e:
            rospy.logerr("Failed to launch Walk state node: {}".format(e))


    def check_valid_transition(self, _current_state, _next_state):
        return _next_state in valid_transitions.get(_current_state, [])

    def change_state_callback(self, msg):
        """Move to the next state if available."""
        desired_state = msg.data
        current_state = scotty_states[self.current_state_key ]

        rospy.loginfo("\n\nRECEIVED")

        valid_transitions = {
            "Ready"  : "Stand",
            "Stand"  : "Walk" ,

        }

        if check_valid_transition(current_state, desired_state):
            rospy.loginfo("Transitioning from {} to {}.".format(current_state,desired_state))
            self.current_state_key = desired_state
            self.start_current_state()
        else:
            rospy.loginfo("Invalid state transition. Request ignored.")


if __name__ == "__main__":
    try:
        controller = MainController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

