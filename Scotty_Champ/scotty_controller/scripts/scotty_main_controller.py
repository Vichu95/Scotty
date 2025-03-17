#!/usr/bin/env python

"""
Project     : Scotty
ROS Package : scotty_controller
Script Name : scotty_main_controller.py
Author      : Vishnudev Kurumbaparambil
Organization: Hochschule Anhalt
Description : The main script that controlls the states of the robot
Usage       : These scripts are run from the scotty_main_controller.launch
"""

import rospy
import subprocess
from std_msgs.msg import String
import webbrowser
import rospkg

######
## STATES
######
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
        rospy.loginfo("Starting the Main Controller for Scotty...")

        ## Open the GUI
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("scotty_controller")
        webbrowser.open(package_path + "/scripts/index.html")

        self.current_state_key = 'Idle'
        self.state_exec_status = 'Wait'
        self.state_already_processed = False

        # Publisher to broadcast current state
        self.state_pub = rospy.Publisher("/scotty_controller/state", String, queue_size=10)
        # Publisher to broadcast logs to the GUI console
        self.console_log_pub = rospy.Publisher("/scotty_controller/console_log", String, queue_size=10)

        # Subscribe to state change command and execution status
        self.state_change_sub = rospy.Subscriber("/scotty_controller/change_state", String, self.change_state_callback)
        self.state_exec_status_sub = rospy.Subscriber("/scotty_controller/state_execution_status", String, self.state_execution_status_callback)

        # Check if it is connected with hardware or simulation
        self.hardware_connected = rospy.get_param("/scotty_controller/hardware_connected", False)
        rospy.loginfo("Hardware connected : {}".format(self.hardware_connected))
        self.mode = "hardware" if self.hardware_connected else "simulation"
        
        # Start the initial state
        print("\n\n")
        self.console_log_pub.publish("INFO    : Controller is loading...")
        self.start_current_state()

    ##
    ## This function handles the calling of the current state functions
    ##
    def start_current_state(self):
        current_state = scotty_states[self.current_state_key]
        rospy.loginfo("Entering state : {}".format(current_state))

        # Publish the current state
        self.state_pub.publish(current_state)
        self.console_log_pub.publish("INFO    : Entering state : " + current_state)
        # self.console_log_pub.publish("ERROR   : Entering state : " + current_state)
        # self.console_log_pub.publish("WARNING : Entering state : " + current_state)
        # self.console_log_pub.publish("SUCCESS : Entering state : " + current_state)

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
            idle_process = subprocess.Popen(["rosrun", "scotty_controller", "scripts/state_controllers/scotty_state_idle.py"])
            idle_process.wait()  # Wait for the Idle state process to complete
            rospy.loginfo("Completed Idle State.")
            self.console_log_pub.publish("INFO    : Completed execution of Idle state")
            self.console_log_pub.publish("INFO    : Going to Ready state shortly")

            ## Going to ready state automatically in 2s
            rospy.sleep(2)
            self.current_state_key = 'Ready'
            self.start_current_state()

        except Exception as e:
            rospy.logerr("Failed to launch Idle state node: {}".format(e))
            self.console_log_pub.publish("ERROR   : Failed execution of Idle state")


    ##############
    #   R E A D Y 
    ##############
    def start_ready_state(self):
        # rospy.loginfo("Scotty is Ready...")
        rospy.sleep(5)
        # Stay in Ready state
        self.start_current_state()


    ##############
    #   D O W N 
    ##############
    def start_down_state(self):
        rospy.loginfo("Launching Down state node...")
        try:
            self.current_state_key = 'Busy'
            self.state_pub.publish("Busy")
            self.console_log_pub.publish("INFO    : Entering state : Busy")

            down_process = subprocess.Popen(["rosrun", "scotty_controller", "scripts/state_controllers/scotty_state_down.py", self.mode])
            down_process.wait()  # Wait for the Down state process to complete

        except Exception as e:
            rospy.logerr("Failed to launch Down state node: {}".format(e))
            self.console_log_pub.publish("ERROR   : Failed execution of Down state")


    ##############
    #   S T A N D 
    ##############
    def start_stand_state(self):
        rospy.loginfo("Launching Stand state node...")
        try:
            self.current_state_key = 'Busy'
            self.state_pub.publish("Busy")
            self.console_log_pub.publish("INFO    : Entering state : Busy")
            stand_process = subprocess.Popen(["rosrun", "scotty_controller", "scripts/state_controllers/scotty_state_stand.py"])
            stand_process.wait()  # Wait for the Stand state process to complete
            rospy.loginfo("Stand node wait finished")

        except Exception as e:
            rospy.logerr("Failed to launch Stand state node: {}".format(e))
            self.console_log_pub.publish("ERROR   : Failed execution of Stand state")


    ##############
    #   W A L K 
    ##############
    def start_walk_state(self):
        rospy.loginfo("Launching Walk state node...")
        try:
            walk_process = subprocess.Popen(["rosrun", "scotty_controller", "scripts/state_controllers/scotty_state_walk.py"])
            walk_process.wait()  # Wait for the Walk state process to complete
            rospy.loginfo("Completed Walk State.")
            self.console_log_pub.publish("SUCCESS : Completed execution of Walk state")

        except Exception as e:
            rospy.logerr("Failed to launch Walk state node: {}".format(e))
            self.console_log_pub.publish("ERROR   : Failed execution of Walk state")

    def stop_walk_state(self):
        rospy.loginfo("Stopping Walk state node...")
        try:
            self.current_state_key = 'Busy'
            self.state_pub.publish("Busy")
            self.console_log_pub.publish("INFO    : Entering state : Busy")
            ## Killing all the nodes that where started as a part of Walk via Champ
            subprocess.Popen(["rosnode", "kill", "/footprint_to_odom_ekf"])
            subprocess.Popen(["rosnode", "kill", "/base_to_footprint_ekf"])
            subprocess.Popen(["rosnode", "kill", "/state_estimator"])
            subprocess.Popen(["rosnode", "kill", "/champ_controller"])
            subprocess.Popen(["rosnode", "kill", "/velocity_smoother"])
            subprocess.Popen(["rosnode", "kill", "/nodelet_manager"])

            rospy.sleep(2)
            rospy.loginfo("Stopped Walk State.")

        except Exception as e:
            rospy.logerr("Failed to launch Walk state node: {}".format(e))
            self.console_log_pub.publish("ERROR   : Failed execution of stopping Walk state")


    ##############
    #   S H U T D O W N 
    ##############
    def start_shutdown_state(self):
        rospy.loginfo("Launching Shutdown state node...")
        try:
            self.current_state_key = 'Busy'
            self.state_pub.publish("Busy")
            self.console_log_pub.publish("INFO    : Entering state : Busy")
            shutdown_process = subprocess.Popen(["rosrun", "scotty_controller", "scripts/state_controllers/scotty_state_shutdown.py"])
            shutdown_process.wait()  # Wait for the Shutdown state process to complete

        except Exception as e:
            rospy.logerr("Failed to launch Shutdown state node: {}".format(e))
            self.console_log_pub.publish("ERROR   : Failed execution of Shutdown state")


    ##############
    #   R E S E T 
    ##############
    def start_reset_state(self):
        rospy.loginfo("Launching Reset state node...")
        try:
            reset_process = subprocess.Popen(["rosrun", "scotty_controller", "scripts/state_controllers/scotty_state_idle.py"])
            reset_process.wait()  # Wait for the Idle state process to complete
            rospy.loginfo("Completed Reset State.")
            self.console_log_pub.publish("INFO    : Completed resetting")

            ## Going to ready state automatically in 3s
            self.console_log_pub.publish("INFO    : Going to Ready state shortly")
            rospy.sleep(3)
            # Subscribe to state change command

            rospy.loginfo("Re-registering the subscribers.")
            self.state_change_sub.unregister()
            self.state_change_sub = rospy.Subscriber("/scotty_controller/change_state", String, self.change_state_callback)
            self.state_exec_status_sub.unregister()
            self.state_exec_status_sub = rospy.Subscriber("/scotty_controller/state_execution_status", String, self.state_execution_status_callback)

            self.current_state_key = 'Ready'
            self.start_current_state()

        except Exception as e:
            rospy.logerr("Failed to launch Reset state node: {}".format(e))
            self.console_log_pub.publish("ERROR   : Failed execution of Reset state")


    ##############
    #  Handling the change of state
    ##############

    def check_valid_transition(self, _current_state, _next_state):
        return _next_state in valid_transitions.get(_current_state, [])

    # Handles the callback to go to next state
    def change_state_callback(self, msg):
        desired_state = msg.data
        current_state = scotty_states[self.current_state_key ]

        print("\n\n")
        rospy.loginfo("Received a state change request.")
        self.state_already_processed = False

        if self.check_valid_transition(current_state, desired_state):
            rospy.loginfo("Transitioning from {} to {}.".format(current_state,desired_state))
            self.console_log_pub.publish("INFO    : " + current_state + " --> " + desired_state )
            
            ##Incase of Walk as previous state, killing all nodes related to this
            if(current_state == 'Walk'):
                self.stop_walk_state()
            
            self.current_state_key = desired_state
            self.start_current_state()
        else:
            rospy.loginfo("Invalid state transition from " + current_state + " to "+ desired_state +". Request ignored.")
            self.console_log_pub.publish("WARNING : Invalid state transition from " + current_state + " to "+ desired_state +". Request ignored")

    ## This callback handles the status of state execution. Updated from state nodes
    def state_execution_status_callback(self, msg):
        status_recevied = msg.data        
        self.state_exec_status = status_recevied
        rospy.loginfo("Received status of execution : {}".format(status_recevied))

        self.state_already_processed = True

        if self.state_exec_status == "Stand_Done":
            self.current_state_key = 'Stand'
            self.state_pub.publish("Stand")
            self.console_log_pub.publish("SUCCESS : Completed execution of Stand state")

        elif self.state_exec_status == "Down_Done":
            self.current_state_key = 'Down'
            self.state_pub.publish("Down")
            self.console_log_pub.publish("SUCCESS : Completed execution of Down state")

        elif self.state_exec_status == "Shutdown_Pose_Done":
            self.current_state_key = 'Shutdown'
            self.state_pub.publish("Shutdown")
            self.console_log_pub.publish("SUCCESS : Shuting down...")

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

