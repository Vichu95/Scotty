#!/usr/bin/env python

import rospy
import subprocess
import time
from std_msgs.msg import String
import webbrowser
import rospkg

## STATE MACHINE
scotty_states = {
    "Idle"      : "Idle",
    "Ready"     : "Ready",
    "Down"      : "Down",
    "Stand"     : "Stand",
    "Walk"      : "Walk",
    "Busy"      : "Busy",
    "Shutdown"  : "Shutdown",
    "Reset"     : "Reset"
}

valid_transitions = {
    scotty_states["Idle"]       : [scotty_states["Ready"], scotty_states["Shutdown"], scotty_states["Reset"]], 
    scotty_states["Ready"]      : [scotty_states["Down"], scotty_states["Stand"], scotty_states["Shutdown"], scotty_states["Reset"]],
    scotty_states["Down"]       : [scotty_states["Stand"], scotty_states["Shutdown"], scotty_states["Reset"]],
    scotty_states["Stand"]      : [scotty_states["Down"], scotty_states["Walk"], scotty_states["Shutdown"], scotty_states["Reset"]],
    scotty_states["Walk"]       : [scotty_states["Stand"], scotty_states["Shutdown"], scotty_states["Reset"]],
    scotty_states["Busy"]       : [scotty_states["Idle"], scotty_states["Ready"], scotty_states["Down"], scotty_states["Stand"], scotty_states["Walk"]],
    scotty_states["Shutdown"]   : [scotty_states["Idle"]],
    scotty_states["Reset"]      : [scotty_states["Idle"], scotty_states["Ready"]]
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
        self.state_exec_status = 'Wait'
        self.state_already_processed = False

        # Publisher to broadcast current state
        self.state_pub = rospy.Publisher("/scotty_controller/state", String, queue_size=10)
        self.console_log_pub = rospy.Publisher("/scotty_controller/console_log", String, queue_size=10)

        # Subscribe to state change command
        self.state_change_sub = rospy.Subscriber("/scotty_controller/change_state", String, self.change_state_callback)
        self.state_exec_status_sub = rospy.Subscriber("/scotty_controller/state_execution_status", String, self.state_execution_status_callback)

        # Start the initial state
        self.start_current_state()

    def start_current_state(self):
        """Start the current state based on the state index."""
        current_state = scotty_states[self.current_state_key]
        rospy.loginfo("Entering state: {}".format(current_state))

        # Publish the current state
        self.state_pub.publish(current_state)
        self.console_log_pub.publish(current_state)

        if(not self.state_already_processed):
            if current_state == "Idle":
                self.start_idle_state()
            elif current_state == "Ready":
                self.start_ready_state()
            elif current_state == "Down":
                self.start_down_state()
            elif current_state == "Stand":
                self.start_stand_state()
            elif current_state == "Walk":
                self.start_walk_state()
            elif current_state == "Shutdown":
                self.start_shutdown_state()
            elif current_state == "Reset":
                self.start_reset_state()
            elif current_state == "Busy":
                i = 0 # Do nothing
            else:
                rospy.logwarn("Unknown state requested: {}".format(current_state))
        else:
            rospy.loginfo("State is already processed")

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
        self.console_log_pub.publish('ready is ready 1')
        self.start_current_state()


    ##############
    #   D O W N 
    ##############
    def start_down_state(self):
        rospy.loginfo("Launching Down state node")
        try:
            self.current_state_key = 'Busy'
            self.state_pub.publish("Busy")
            stand_process = subprocess.Popen(["rosrun", "scotty_config", "state_down_controller.py"])
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
            self.console_log_pub.publish('in stand mode')
            self.current_state_key = 'Busy'
            self.state_pub.publish("Busy")
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

    def stop_walk_state(self):
        rospy.loginfo("Stopping Walk state node")
        try:
            self.current_state_key = 'Busy'
            self.state_pub.publish("Busy")
            subprocess.Popen(["rosnode", "kill", "/footprint_to_odom_ekf"])
            subprocess.Popen(["rosnode", "kill", "/base_to_footprint_ekf"])
            subprocess.Popen(["rosnode", "kill", "/state_estimator"])
            subprocess.Popen(["rosnode", "kill", "/champ_controller"])
            subprocess.Popen(["rosnode", "kill", "/velocity_smoother"])
            subprocess.Popen(["rosnode", "kill", "/nodelet_manager"])

            rospy.sleep(2)
            rospy.loginfo("Walk state completly closed.")

        except Exception as e:
            rospy.logerr("Failed to launch Walk state node: {}".format(e))


    ##############
    #   S H U T D O W N 
    ##############
    def start_shutdown_state(self):
        rospy.loginfo("Launching Shutdown state node")
        try:
            self.current_state_key = 'Busy'
            self.state_pub.publish("Busy")
            stand_process = subprocess.Popen(["rosrun", "scotty_config", "state_shutdown_controller.py"])
            stand_process.wait()  # Wait for the Shutdown state process to complete
            rospy.loginfo("Shutdown state completed.")

        except Exception as e:
            rospy.logerr("Failed to launch Shutdown state node: {}".format(e))


    ##############
    #   R E S E T 
    ##############
    def start_reset_state(self):
        rospy.loginfo("Launching Reset state node...")
        try:
            reset_process = subprocess.Popen(["rosrun", "scotty_config", "state_idle_startup.py"])
            reset_process.wait()  # Wait for the Idle state process to complete
            rospy.loginfo("Reset state completed.")

            ## Going to ready state automatically in 3s
            rospy.sleep(3)
            # Subscribe to state change command

            print("\n\nRegistering agian")
            self.state_change_sub.unregister()
            self.state_change_sub = rospy.Subscriber("/scotty_controller/change_state", String, self.change_state_callback)
            self.state_exec_status_sub.unregister()
            self.state_exec_status_sub = rospy.Subscriber("/scotty_controller/state_execution_status", String, self.state_execution_status_callback)

            self.current_state_key = 'Ready'
            self.start_current_state()

        except Exception as e:
            rospy.logerr("Failed to launch Reset state node: {}".format(e))


    def check_valid_transition(self, _current_state, _next_state):
        return _next_state in valid_transitions.get(_current_state, [])

    def change_state_callback(self, msg):
        """Move to the next state if available."""
        desired_state = msg.data
        current_state = scotty_states[self.current_state_key ]

        rospy.loginfo("\n\nRECEIVED")
        self.state_already_processed = False

        if self.check_valid_transition(current_state, desired_state):
            rospy.loginfo("Transitioning from {} to {}.".format(current_state,desired_state))
            
            ##Incase of Walk as previous state, killing all nodes related to this
            if(current_state == 'Walk'):
                self.stop_walk_state()
            
            self.current_state_key = desired_state
            self.start_current_state()
        else:
            rospy.loginfo("Invalid state transition. Request ignored.")

    ## This callback stores the information about the status of state execution
    def state_execution_status_callback(self, msg):
        status_recevied = msg.data        
        self.state_exec_status = status_recevied
        rospy.loginfo("\n\nRECEIVED stateeuss exection {}".format(status_recevied))

        self.state_already_processed = True

        if self.state_exec_status == "Stand_Done":
            self.current_state_key = 'Stand'
            self.state_pub.publish("Stand")
            print("\n\nstate stand execuion finishedd for finishing execution")
        elif self.state_exec_status == "Down_Done":
            self.current_state_key = 'Down'
            self.state_pub.publish("Down")
            print("\n\nstate Down execuion finishedd for finishing execution")
        elif self.state_exec_status == "Shutdown_Done":
            self.current_state_key = 'Shutdown'
            self.state_pub.publish("Shutdown")

            print("\n\nstate Shutdown execuion finishedd for finishing execution")
            # Stop the ROS nodes cleanly
            rospy.signal_shutdown("Shutting down all nodes...")
    
        else:
            print("Waiting for finishing execution")

if __name__ == "__main__":
    try:
        controller = MainController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

