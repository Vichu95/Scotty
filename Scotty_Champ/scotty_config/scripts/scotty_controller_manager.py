#!/usr/bin/env python
 
import rospy
from controller_manager_msgs.srv import ListControllers, LoadController, SwitchController
 
class ScottyControllerManager:
    def __init__(self):
        # Initialize the services for controller management
        rospy.wait_for_service("/controller_manager/list_controllers")
        rospy.wait_for_service("/controller_manager/load_controller")
        rospy.wait_for_service("/controller_manager/switch_controller")
        
        self.list_controllers_srv = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
        self.load_controller_srv = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)
        self.switch_controller_srv = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
 
    def load_and_start_controller(self, controller_name):
        # Check if the controller is already loaded and running
        try:
            rospy.loginfo("Checking if {} is already loaded...".format(controller_name))
            list_response = self.list_controllers_srv()
            controller_found = False
            controller_running = False
            
            for controller in list_response.controller:
                if controller.name == controller_name:
                    controller_found = True
                    if controller.state == "running":
                        controller_running = True
                    break
            
            if controller_found:
                if controller_running:
                    rospy.loginfo("{} is already loaded and running.".format(controller_name))
                    return True
                else:
                    rospy.loginfo("{} is loaded but not running. Starting it now...".format(controller_name))
                    return self.start_controller(controller_name)
            else:
                rospy.loginfo("{} not loaded. Loading and starting it now...".format(controller_name))
                return self.load_and_start_new_controller(controller_name)
 
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return False
 
    def load_and_start_new_controller(self, controller_name):
        # Load the controller
        try:
            rospy.loginfo("Loading {}...".format(controller_name))
            load_response = self.load_controller_srv(controller_name)
            if load_response.ok:
                rospy.loginfo("{} loaded successfully.".format(controller_name))
            else:
                rospy.logerr("Failed to load {}.".format(controller_name))
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return False
 
        # Start the controller
        return self.start_controller(controller_name)
 
    def start_controller(self, controller_name):
        try:
            rospy.loginfo("Starting {}...".format(controller_name))
            switch_response = self.switch_controller_srv(
                start_controllers=[controller_name],
                stop_controllers=[],
                strictness=2
            )
            if switch_response.ok:
                rospy.loginfo("{} started successfully.".format(controller_name))
                return True
            else:
                rospy.logerr("Failed to start {}.".format(controller_name))
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return False